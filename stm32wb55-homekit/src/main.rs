//! BLE Eddystone URL beacon example.
#![no_main]
#![no_std]
#![allow(non_snake_case)]

use panic_rtt_target as _;
// use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

extern crate stm32wb_hal as hal;

use core::{convert::TryFrom, time::Duration};

use cortex_m_rt::{entry, exception};
use nb::block;

use hal::{
    flash::FlashExt,
    prelude::*,
    rcc::{
        ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, RfWakeupClock, RtcClkSrc,
        StopWakeupClock, SysClkSrc,
    },
    tl_mbox::{lhci::LhciC1DeviceInformationCcrp, shci::ShciBleInitCmdParam, TlMbox},
};

use bluetooth_hci::{
    event::{command::ReturnParameters, Event},
    host::{uart::Packet, AdvertisingFilterPolicy, EncryptionKey, Hci, OwnAddressType},
    BdAddr,
};

use ble::{perform_command, receive_event, setup_coprocessor, Characteristic, RadioCopro};
use hap::{GattFormat, HapCharacteristic, HapProperties, HapService};
use homekit_ble::{
    tlv::{Tlv, Value},
    HapPdu, HapResponse, HapStatus, OpCode,
};
use stm32wb55::{
    event::{AttReadPermitRequest, AttributeHandle, GattAttributeModified, Stm32Wb5xEvent},
    gap::{
        AdvertisingDataType, AdvertisingType, AuthenticationRequirements, Commands as GapCommands,
        DiscoverableParameters, LocalName, OutOfBandAuthentication, Pin, Role,
    },
    gatt::{CharacteristicProperty, Commands as GattCommads, UpdateCharacteristicValueParameters},
    hal::{Commands as HalCommands, ConfigData, PowerLevel},
};
use uuid::{
    UUID_ACCESSORY_INFORMATION, UUID_ACCESSORY_INFORMATION_FIRMWARE_REVISION,
    UUID_ACCESSORY_INFORMATION_HARDWARE_REVISION, UUID_ACCESSORY_INFORMATION_IDENTIFY,
    UUID_ACCESSORY_INFORMATION_MANUFACTURER, UUID_ACCESSORY_INFORMATION_MODEL,
    UUID_ACCESSORY_INFORMATION_SERIAL_NUMBER, UUID_CHARACTERISTIC_NAME, UUID_CHARACTERISTIC_ON,
    UUID_LIGHTBULB_SERVICE, UUID_PAIRING_FEATURES, UUID_PAIRING_PAIRINGS, UUID_PAIRING_SERVICE,
    UUID_PAIRING_SETUP, UUID_PAIRING_VERIFY, UUID_PROTOCOL_INFORMATION, UUID_SERVICE_SIGNATURE,
    UUID_VERSION_CHARACTERISTIC,
};

mod ble;
mod hap;
mod uuid;

/// Advertisement interval in milliseconds.
const ADV_INTERVAL_MS: u64 = 250;

const BT_NAME: &[u8] = b"hokt";
const BLE_GAP_DEVICE_NAME_LENGTH: u8 = BT_NAME.len() as u8;

#[entry]
fn entry() -> ! {
    //rtt_init_print!(BlockIfFull, 4096);
    rtt_init_print!(NoBlockSkip, 4096);
    run();

    loop {
        continue;
    }
}

fn run() {
    let dp = hal::device::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    rcc.set_stop_wakeup_clock(StopWakeupClock::HSI16);

    // Fastest clock configuration.
    // * External low-speed crystal is used (LSE)
    // * 32 MHz HSE with PLL
    // * 64 MHz CPU1, 32 MHz CPU2
    // * 64 MHz for APB1, APB2
    // * HSI as a clock source after wake-up from low-power mode
    let clock_config = Config::new(SysClkSrc::Pll(PllSrc::Hse(HseDivider::NotDivided)))
        .with_lse()
        .cpu1_hdiv(HDivider::NotDivided)
        .cpu2_hdiv(HDivider::Div2)
        .apb1_div(ApbDivider::NotDivided)
        .apb2_div(ApbDivider::NotDivided)
        .pll_cfg(PllConfig {
            m: 2,
            n: 12,
            r: 3,
            q: Some(4),
            p: Some(3),
        })
        .rtc_src(RtcClkSrc::Lse)
        .rf_wkp_sel(RfWakeupClock::Lse);

    let mut rcc = rcc.apply_clock_config(clock_config, &mut dp.FLASH.constrain().acr);

    rprintln!("Boot");

    // RTC is required for proper operation of BLE stack
    let _rtc = hal::rtc::Rtc::rtc(dp.RTC, &mut rcc);

    let mut ipcc = dp.IPCC.constrain();
    let mbox = TlMbox::tl_init(&mut rcc, &mut ipcc);

    let config = ShciBleInitCmdParam {
        p_ble_buffer_address: 0,
        ble_buffer_size: 0,
        num_attr_record: 100,
        num_attr_serv: 10,
        attr_value_arr_size: 3500, //2788,
        num_of_links: 8,
        extended_packet_length_enable: 1,
        pr_write_list_size: 0x3A,
        mb_lock_count: 0x79,
        att_mtu: 312,
        slave_sca: 500,
        master_sca: 0,
        ls_source: 1,
        max_conn_event_length: 0xFFFFFFFF,
        hs_startup_time: 0x148,
        viterbi_enable: 1,
        ll_only: 0,
        hw_version: 0,
    };

    setup_coprocessor(config, ipcc, mbox);

    // enable interrupts -> interrupts are enabled in Ipcc::init(), which is called TlMbox::tl_init

    // Boot CPU2
    hal::pwr::set_cpu2(true);

    let ready_event = block!(receive_event());

    rprintln!("Received packet: {:?}", ready_event);

    rprintln!("Resetting processor...");

    let reset_response = perform_command(|rc| rc.reset()).expect("Failed to reset processor");

    rprintln!("Received packet: {:?}", reset_response);

    let homekit_accessory = init_gap_and_gatt().expect("Failed to initialize GAP and GATT");

    rprintln!("Succesfully initialized GAP and GATT");

    init_homekit().expect("Failed to initialize homekit setup");

    rprintln!("Succesfully initialized Homekit");

    loop {
        let response = block!(receive_event());

        rprintln!("Received event: {:x?}", response);

        if let Ok(Packet::Event(event)) = response {
            match event {
                Event::DisconnectionComplete(_state) => {
                    // Enter advertising mode again
                    // Put the device in a connectable mode
                    perform_command(|rc| {
                        rc.set_discoverable(&DISCOVERY_PARAMS)
                            .map_err(|_| nb::Error::Other(()))
                    })
                    .expect("Failed to enable discoverable mode again");

                    perform_command(|rc| {
                        rc.update_advertising_data(&ADVERTISING_DATA[..])
                            .map_err(|_| nb::Error::Other(()))
                    })
                    .expect("Failed to update advertising data");
                }
                other => homekit_accessory.handle_event(&other),
            }
        }
    }
}

struct HapAccessory {
    protocol_service: ProtocolService,

    pairing_service: PairingService,
}

impl HapAccessory {
    fn handle_event(&self, event: &Event<Stm32Wb5xEvent>) {
        if let Event::Vendor(stm_event) = event {
            match stm_event {
                Stm32Wb5xEvent::GattAttributeModified(modified) => {
                    rprintln!("Handling write to attribute {:?}", modified.attr_handle);

                    if self.protocol_service.contains_handle(modified.attr_handle) {
                        self.protocol_service
                            .handle_attribute_modified(modified)
                            .expect("Failed to handle AttributeModified event");
                    } else if self.pairing_service.contains_handle(modified.attr_handle) {
                        self.pairing_service
                            .handle_attribute_modified(modified)
                            .expect("Failed to handle AttributeModified event");
                    }
                }
                Stm32Wb5xEvent::AttReadPermitRequest(AttReadPermitRequest {
                    conn_handle,
                    attribute_handle: _,
                    offset: _,
                }) => {
                    // TODO: Check if allowed
                    perform_command(|rc| rc.allow_read(*conn_handle))
                        .expect("Failed to allow read");
                }
                // Ignore other events
                _ => {}
            }
        }
    }
}

#[exception]
fn DefaultHandler(irqn: i16) -> ! {
    panic!("Unhandled IRQ: {}", irqn);
}

fn get_bd_addr() -> BdAddr {
    let mut bytes = [0u8; 6];

    let lhci_info = LhciC1DeviceInformationCcrp::new();
    bytes[0] = (lhci_info.uid64 & 0xff) as u8;
    bytes[1] = ((lhci_info.uid64 >> 8) & 0xff) as u8;
    bytes[2] = ((lhci_info.uid64 >> 16) & 0xff) as u8;
    bytes[3] = lhci_info.device_type_id;
    bytes[4] = (lhci_info.st_company_id & 0xff) as u8;
    bytes[5] = (lhci_info.st_company_id >> 8 & 0xff) as u8;

    BdAddr(bytes)
}

fn init_gap_and_gatt() -> Result<HapAccessory, ()> {
    let response = perform_command(|rc: &mut RadioCopro| {
        rc.write_config_data(&ConfigData::public_address(get_bd_addr()).build())
    })?;

    rprintln!("Response to write_config_data: {:?}", response);

    perform_command(|rc| {
        rc.write_config_data(&ConfigData::random_address(get_random_addr()).build())
    })?;

    perform_command(|rc| rc.write_config_data(&ConfigData::identity_root(&get_irk()).build()))?;

    perform_command(|rc| rc.write_config_data(&ConfigData::encryption_root(&get_erk()).build()))?;

    perform_command(|rc| rc.set_tx_power_level(PowerLevel::ZerodBm))?;

    perform_command(|rc| rc.init_gatt())?;

    let return_params =
        perform_command(|rc| rc.init_gap(Role::PERIPHERAL, false, BLE_GAP_DEVICE_NAME_LENGTH))?;

    let (service_handle, dev_name_handle, appearence_handle) = if let ReturnParameters::Vendor(
        stm32wb55::event::command::ReturnParameters::GapInit(stm32wb55::event::command::GapInit {
            service_handle,
            dev_name_handle,
            appearance_handle,
            ..
        }),
    ) = return_params
    {
        (service_handle, dev_name_handle, appearance_handle)
    } else {
        rprintln!("Unexpected response to init_gap command");
        return Err(());
    };

    perform_command(|rc| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: dev_name_handle,
            offset: 0,
            value: BT_NAME,
        })
        .map_err(|_| nb::Error::Other(()))
    })?;

    let appearance_characteristic = Characteristic {
        service: service_handle,
        characteristic: appearence_handle,
        max_len: 4,
    };

    appearance_characteristic.set_value(&[0x80, 0x00])?;

    // Setup authentication
    perform_command(|rc| {
        rc.set_authentication_requirement(&AuthenticationRequirements {
            mitm_protection_required: false,
            out_of_band_auth: OutOfBandAuthentication::Disabled,
            encryption_key_size_range: (16, 16),
            fixed_pin: Pin::Requested,
            bonding_required: false,
        })
        .map_err(|_| nb::Error::Other(()))
    })?;

    // Acessory information service
    rprintln!("Accessory information service");

    //cx.next_service = BleServices::AccessoryInformation;
    let accessory_service = HapService::new(UUID_ACCESSORY_INFORMATION, 24, 1)?;

    // add the

    let _information_identify_characteristic = HapCharacteristic::build(
        &accessory_service,
        2,
        UUID_ACCESSORY_INFORMATION_IDENTIFY,
        CharacteristicProperty::WRITE | CharacteristicProperty::READ,
        HapProperties::WRITE,
        GattFormat::Bool,
        1,
    )?;

    let information_manufacturer_characteristic = HapCharacteristic::build(
        &accessory_service,
        3,
        UUID_ACCESSORY_INFORMATION_MANUFACTURER,
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        HapProperties::SECURE_READ,
        GattFormat::String,
        64,
    )?;
    //information_manufacturer_characteristic.set_value(b"Dominik Corp.\0")?;

    let information_model_characteristic = HapCharacteristic::build(
        &accessory_service,
        4,
        UUID_ACCESSORY_INFORMATION_MODEL,
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        HapProperties::SECURE_READ,
        GattFormat::String,
        10,
    )?;
    //information_model_characteristic.set_value(b"M001\0")?;

    let information_name_characteristic = HapCharacteristic::build(
        &accessory_service,
        5,
        UUID_CHARACTERISTIC_NAME,
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        HapProperties::SECURE_READ,
        GattFormat::String,
        10,
    )?;
    //information_name_characteristic.set_value(BT_NAME)?;

    let information_serial_number_characteristic = HapCharacteristic::build(
        &accessory_service,
        6,
        UUID_ACCESSORY_INFORMATION_SERIAL_NUMBER,
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        HapProperties::SECURE_READ,
        GattFormat::String,
        15,
    )?;
    //information_serial_number_characteristic.set_value(b"S12345\0")?;

    let information_firmware_revision_characteristic = HapCharacteristic::build(
        &accessory_service,
        7,
        UUID_ACCESSORY_INFORMATION_FIRMWARE_REVISION,
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        HapProperties::SECURE_READ,
        GattFormat::String,
        10,
    )?;
    //information_firmware_revision_characteristic.set_value(b"1.0.0\0")?;

    let information_hardware_revision_characteristic = HapCharacteristic::build(
        &accessory_service,
        8,
        UUID_ACCESSORY_INFORMATION_HARDWARE_REVISION,
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        HapProperties::SECURE_READ,
        GattFormat::String,
        10,
    )?;
    //information_hardware_revision_characteristic.set_value(b"1.0.0\0")?;

    let protocol_service = ProtocolService::create_ble()?;

    let pairing_service = PairingService::create_ble()?;

    // Create light bulb service

    let light_bulb_service = HapService::new(UUID_LIGHTBULB_SERVICE, 20, 0x30)?;

    // signature

    let lb_signature = HapCharacteristic::build(
        &light_bulb_service,
        0x32,
        UUID_SERVICE_SIGNATURE,
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        HapProperties::SECURE_READ,
        GattFormat::Data,
        64,
    )?;

    let lb_name = HapCharacteristic::build(
        &light_bulb_service,
        0x31,
        UUID_CHARACTERISTIC_NAME,
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        HapProperties::SECURE_READ,
        GattFormat::String,
        64,
    )?;

    let lb_on = HapCharacteristic::build(
        &light_bulb_service,
        0x33,
        UUID_CHARACTERISTIC_ON,
        CharacteristicProperty::READ
            | CharacteristicProperty::WRITE
            | CharacteristicProperty::NOTIFY,
        HapProperties::SECURE_READ | HapProperties::SECURE_WRITE | HapProperties::NOTIFY_CONNECTED,
        GattFormat::Bool,
        64,
    )?;

    Ok(HapAccessory {
        protocol_service,
        pairing_service,
    })
}

struct PairingService {
    service: HapService,

    setup: HapCharacteristic,
    verify: HapCharacteristic,
    features: HapCharacteristic,
    pairings: HapCharacteristic,

    pairing_state: PairingSetupState,
}

impl PairingService {
    fn create_ble() -> Result<Self, ()> {
        // Add Pairing service
        rprintln!("Pairing service");
        let pairing_service = HapService::new(UUID_PAIRING_SERVICE, 15, 0x20)?;

        // TODO: not hardcoded value here

        let pair_setup = HapCharacteristic::build(
            &pairing_service,
            0x22,
            UUID_PAIRING_SETUP,
            CharacteristicProperty::READ | CharacteristicProperty::WRITE,
            HapProperties::READ | HapProperties::WRITE,
            GattFormat::Data,
            512,
        )?;

        let pair_verify = HapCharacteristic::build(
            &pairing_service,
            0x23,
            UUID_PAIRING_VERIFY,
            CharacteristicProperty::READ | CharacteristicProperty::WRITE,
            HapProperties::READ | HapProperties::WRITE,
            GattFormat::Data,
            512,
        )?;

        let pairing_features = HapCharacteristic::build(
            &pairing_service,
            0x24,
            UUID_PAIRING_FEATURES,
            CharacteristicProperty::READ | CharacteristicProperty::WRITE,
            HapProperties::READ, // | HapProperties::WRITE,
            GattFormat::Uint8,
            512,
        )?;
        let pairing_pairings = HapCharacteristic::build(
            &pairing_service,
            0x25,
            UUID_PAIRING_PAIRINGS,
            CharacteristicProperty::READ | CharacteristicProperty::WRITE,
            HapProperties::SECURE_READ | HapProperties::SECURE_WRITE,
            GattFormat::Data,
            512,
        )?;

        Ok(Self {
            service: pairing_service,
            setup: pair_setup,
            verify: pair_verify,
            features: pairing_features,
            pairings: pairing_pairings,
            pairing_state: PairingSetupState::M1,
        })
    }

    /// Check if a BLE attribute handle is part of this service
    fn contains_handle(&self, handle: AttributeHandle) -> bool {
        self.service.contains_handle(handle)
    }

    fn get_characteristic(&self, instance_id: u16) -> Option<&HapCharacteristic> {
        if instance_id == self.setup.instance_id {
            Some(&self.setup)
        } else if instance_id == self.verify.instance_id {
            Some(&self.verify)
        } else if instance_id == self.features.instance_id {
            Some(&self.features)
        } else if instance_id == self.pairings.instance_id {
            Some(&self.pairings)
        } else {
            // Unsupported characteristic ID
            rprintln!(
                "Characteristic with ID {} is not part of this service.",
                instance_id
            );
            None
        }
    }

    /// Handle a BLE event for this service
    fn handle_attribute_modified(&self, modified: &GattAttributeModified) -> Result<(), ()> {
        // Try to parse a HAP PDU
        if let Ok(HapPdu::Request(pdu)) = HapPdu::parse(modified.data()) {
            rprintln!("PDU: {:#x?}", pdu);

            match pdu.op_code {
                OpCode::CharacteristicSignatureRead => {
                    // Signature for Protocol Service Signature Characteristic
                    let characteristic = self.get_characteristic(pdu.char_id).ok_or(())?;

                    characteristic_signature_read(pdu.tid, characteristic, &self.service)?;
                }
                OpCode::CharacteristicRead => {
                    if pdu.char_id == self.features.instance_id {
                        // Read of the Feature Characteristic

                        // response is 0 (this means no)
                        let mut response_data = [0u8; 3];

                        let tlv = Tlv::new(0x01, 0u8);

                        tlv.write_into(&mut response_data);

                        let response =
                            HapResponse::new(pdu.tid, HapStatus::Success, &response_data);

                        // we now have to write the property with the response

                        let mut resp_buff = [0u8; 50];

                        response
                            .write_into(&mut resp_buff)
                            .expect("Failed to HAP Response");

                        // This meas we have to send a xxx event
                        self.features
                            .set_value(&resp_buff[..response.size()])
                            .expect("Failed to set value for ServiceSignatureRead");
                    } else {
                        rprintln!(
                            "Characteristic ID mismatch: self.id={}, pdu.char_id={}",
                            self.features.instance_id,
                            pdu.char_id
                        );
                    }
                }
                OpCode::CharacteristicWrite => {
                    if pdu.char_id == self.setup.instance_id {
                        // Write to the setup characteristic

                        if let Some(data) = pdu.data {
                            let mut write_with_response = false;

                            let mut body: Option<&[u8]> = None;

                            for tlv in Tlv::parse(data) {
                                // The type of these TLV should be one of the type specified in
                                // the interface specification in Table 7-10
                                match tlv.tlv_type {
                                    // HAP-Param-Value
                                    0x1 => match tlv.value {
                                        Value::Bytes(value) => {
                                            body = Some(value);
                                        }
                                        _ => rprintln!("Unexpected TLV body!"),
                                    },
                                    // HAP-Param-Return-Response
                                    0x9 => {
                                        write_with_response = true;
                                    }
                                    _ => {
                                        rprintln!("Unknown TLV in Characteristic-Write");
                                    }
                                }
                            }

                            if let Some(body) = body {
                                let mut state = None;
                                let mut method = None;

                                for tlv in Tlv::parse(body) {
                                    match tlv.tlv_type {
                                        // PairingMethod
                                        0x0 => {
                                            method = Some(tlv);
                                        }
                                        0x6 => {
                                            state = Some(tlv);
                                        }
                                        // Ignore other TLVs,
                                        _ => (),
                                    }
                                }

                                if state.is_some() && method.is_some() {
                                    let method = PairingMethod::try_from(method.unwrap())
                                        .expect("Error parsing pairing method");

                                    rprintln!("Pairing State: {:?}", state.unwrap().value);
                                    rprintln!("Pairing Method: {:?}", method);
                                }
                            }
                        }
                    }
                }

                // Ignore other op codes
                _ => {}
            }
        } else {
            rprintln!("Failed to parse HAP PDU.");
        }

        Ok(())
    }
}

enum PairingSetupState {
    M1,
    M2,
    M3,
    M4,
    M5,
    M6,
}

/// Method used for pairing.
///
/// Encoded as a TLV with type 0x00, and integer format.
#[derive(Debug)]
enum PairingMethod {
    PairSetup,
    PairSetupWithAuth,
    PairVerify,
    AddPairing,
    RemovePairing,
    ListPairings,
    Reserved,
}

impl TryFrom<Tlv<'_>> for PairingMethod {
    type Error = TlvParseError;

    fn try_from(value: Tlv) -> Result<Self, Self::Error> {
        if value.tlv_type != 0x00 {
            return Err(TlvParseError::TypeMismatch);
        }

        let int_value = match value.value {
            Value::Bytes(data) => data[0],
            _ => return Err(TlvParseError::DataMismatch),
        };

        let method = match int_value {
            0 => PairingMethod::PairSetup,
            1 => PairingMethod::PairSetupWithAuth,
            2 => PairingMethod::PairVerify,
            3 => PairingMethod::AddPairing,
            4 => PairingMethod::RemovePairing,
            5 => PairingMethod::ListPairings,
            _ => PairingMethod::Reserved,
        };

        Ok(method)
    }
}

#[derive(Debug)]
enum TlvParseError {
    TypeMismatch,
    DataMismatch,
}

// handle pairing
fn handle_setup() {
    // The first step is a HAP Characteristic Write-with-Response,
    // as described in 7.3.5.5
    //
    // the body of the Characteristic Write contains two TLVs,
    // one with type 1 (HAP-Param-Value), containing the paramaters,
    // one with type 9 (HAP-Param-Return-Response), indicating that the Write has a response

    // we should receive type_state
    // we should receive type_method

    // optionally: type_flags, when separate optional authentication procedure is used
}

struct ProtocolService {
    service: HapService,

    version: HapCharacteristic,

    signature: HapCharacteristic,
}

impl ProtocolService {
    /// Create the necessary GATT services
    /// and characteristics for this service.
    fn create_ble() -> Result<Self, ()> {
        // Protocol information service

        rprintln!("Protocol information service");

        let protocol_information_service = HapService::new(UUID_PROTOCOL_INFORMATION, 9, 0x10)?;

        let protocol_service_signature = HapCharacteristic::build(
            &protocol_information_service,
            0x11,
            UUID_SERVICE_SIGNATURE,
            CharacteristicProperty::READ | CharacteristicProperty::WRITE,
            HapProperties::SECURE_READ,
            GattFormat::Data,
            100,
        )?;

        // Indicate that the protocol service support configuration (7.4.3, p. 121, HAP Specification)
        //service_signature_characteristic.set_value(&[0x04, 0x00])?;

        let protocol_version_characteristic = HapCharacteristic::build(
            &protocol_information_service,
            0x12,
            UUID_VERSION_CHARACTERISTIC,
            CharacteristicProperty::READ | CharacteristicProperty::WRITE,
            HapProperties::SECURE_READ,
            GattFormat::String,
            100,
        )?;

        //protocol_version_characteristic.set_value(b"2.2.0\0")?;

        Ok(Self {
            service: protocol_information_service,
            version: protocol_version_characteristic,
            signature: protocol_service_signature,
        })
    }

    /// Check if a BLE attribute handle is part of this service
    fn contains_handle(&self, handle: AttributeHandle) -> bool {
        self.service.contains_handle(handle)
    }

    /// Handle a BLE event for this service
    fn handle_attribute_modified(&self, modified: &GattAttributeModified) -> Result<(), ()> {
        // Try to parse a HAP PDU
        if let Ok(HapPdu::Request(pdu)) = HapPdu::parse(modified.data()) {
            rprintln!("PDU: {:#x?}", pdu);

            match pdu.op_code {
                OpCode::ServiceSignatureRead => {
                    // Handle read of Protocol Service Signature
                    if pdu.char_id == self.service.instance_id {
                        // We don't link to any services, so the LinkedSvc TLV is not used

                        // The properties of this service are that it support configuration
                        // -> 0x0004

                        let svc_properties = Tlv::new(0xf, 0x04u16);
                        let linked_svc = Tlv::new(0x10, &[][..]);

                        let mut response_data = [0u8; 6];
                        let mut offset = 0;

                        offset += svc_properties.write_into(&mut response_data);

                        offset += linked_svc.write_into(&mut response_data[offset..]);

                        let response =
                            HapResponse::new(pdu.tid, HapStatus::Success, &response_data[offset..]);

                        // we now have to write the property with the response

                        let mut resp_buff = [0u8; 50];

                        response
                            .write_into(&mut resp_buff)
                            .expect("Failed to HAP Response");

                        // This meas we have to send a xxx event
                        self.signature
                            .set_value(&resp_buff[..response.size()])
                            .expect("Failed to set value for ServiceSignatureRead");
                    } else {
                        // Not sure
                    }
                }
                OpCode::CharacteristicSignatureRead => {
                    // Signature for Protocol Service Signature Characteristic
                    let characteristic = if pdu.char_id == self.signature.instance_id {
                        &self.signature
                    } else if pdu.char_id == self.version.instance_id {
                        &self.version
                    } else {
                        // Unsupported characteristic ID
                        rprintln!(
                            "Characteristic with ID {} is not part of this service.",
                            pdu.char_id
                        );
                        return Err(());
                    };

                    characteristic_signature_read(pdu.tid, characteristic, &self.service)?;
                }
                // Ignore other op codes
                _ => {}
            }
        } else {
            rprintln!("Failed to parse HAP PDU.");
        }

        Ok(())
    }
}

fn characteristic_signature_read(
    tid: u8,
    characteristic: &HapCharacteristic,
    service: &HapService,
) -> Result<(), ()> {
    let mut response_data = [0u8; 53];
    let characteristic_uuid = Tlv::new(0x04, &characteristic.uuid[..]);
    let service_uuid = Tlv::new(0x06, &service.uuid[..]);

    let mut offset = 0;

    // characteristic type
    offset += characteristic_uuid.write_into(&mut response_data);

    // service id
    offset += Tlv::new(0x07, service.instance_id).write_into(&mut response_data[offset..]);

    // service type
    offset += service_uuid.write_into(&mut response_data[offset..]);

    // properties
    offset +=
        Tlv::new(0x0a, characteristic.properties.bits()).write_into(&mut response_data[offset..]);

    let mut gatt_format = [0u8; 7];

    // Formatj
    gatt_format[0] = characteristic.format as u8;

    gatt_format[2..4].copy_from_slice(&(characteristic.unit as u16).to_le_bytes());

    // namespace
    gatt_format[4] = 1;

    // GATT Format
    offset += Tlv::new(0x0C, &gatt_format[..]).write_into(&mut response_data[offset..]);

    assert_eq!(
        offset,
        response_data.len(),
        "Error creating HAP response PDU"
    );

    let response = HapResponse::new(tid, HapStatus::Success, &response_data);

    // we now have to write the property with the response

    let mut resp_buff = [0u8; 70];

    response
        .write_into(&mut resp_buff)
        .expect("Failed to build HAP Response");

    // This meas we have to send a xxx event
    characteristic
        .set_value(&resp_buff[..response.size()])
        .expect("Failed to set value for CharacteristicSignatureRead");

    Ok(())
}

fn get_random_addr() -> BdAddr {
    let mut bytes = [0u8; 6];

    let lhci_info = LhciC1DeviceInformationCcrp::new();
    bytes[0] = (lhci_info.uid64 & 0xff) as u8;
    bytes[1] = ((lhci_info.uid64 >> 8) & 0xff) as u8;
    bytes[2] = ((lhci_info.uid64 >> 16) & 0xff) as u8;
    bytes[3] = 0;
    bytes[4] = 0x6E;
    bytes[5] = 0xED;

    BdAddr(bytes)
}

const BLE_CFG_IRK: [u8; 16] = [
    0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
];
const BLE_CFG_ERK: [u8; 16] = [
    0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21, 0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21,
];

fn get_irk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_IRK)
}

fn get_erk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_ERK)
}

const DISCOVERY_PARAMS: DiscoverableParameters = DiscoverableParameters {
    advertising_type: AdvertisingType::ConnectableUndirected,
    advertising_interval: Some((
        Duration::from_millis(ADV_INTERVAL_MS),
        Duration::from_millis(ADV_INTERVAL_MS),
    )),
    address_type: OwnAddressType::Public,
    filter_policy: AdvertisingFilterPolicy::AllowConnectionAndScan,
    local_name: Some(LocalName::Complete(BT_NAME)),
    advertising_data: &[],
    conn_interval: (None, None),
};

const ADVERTISING_DATA: [u8; 19] = [
    0x12, // Length
    0xff, // Manufacturer Data
    0x4c, 0x00, // Apple ID
    0x06, // Type
    0x2D, // STL
    //0x31,
    0x01, // SF
    0x44, 0x75, 0x26, 0x44, 0x58, 0xA3, // Device ID
    0x05, 0x00, // ACID G - Light Bulb
    0x01, 0x00, // GSN
    0x1,  // Configuration number
    0x2,  // CV
          //0x00, 0x00, 0x00, 0x00, // Secure Hash,
];

fn init_homekit() -> Result<(), ()> {
    // Disable scan response
    perform_command(|rc: &mut RadioCopro| {
        rc.le_set_scan_response_data(&[])
            .map_err(|_| nb::Error::Other(()))
    })?;

    // Put the device in a connectable mode
    perform_command(|rc| {
        rc.set_discoverable(&DISCOVERY_PARAMS)
            .map_err(|_| nb::Error::Other(()))
    })?;

    // accessory category: lightbulb (-> lighting 5) (acid)

    perform_command(|rc| {
        rc.update_advertising_data(&ADVERTISING_DATA[..])
            .map_err(|_| nb::Error::Other(()))
    })?;

    perform_command(|rc| {
        let flags = [2, AdvertisingDataType::Flags as u8, 0x4 | 0x2];

        rc.update_advertising_data(&flags[..])
            .map_err(|_| nb::Error::Other(()))
    })?;

    Ok(())
}
