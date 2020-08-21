//! BLE Eddystone URL beacon example.
#![no_main]
#![no_std]
#![allow(non_snake_case)]

use panic_rtt_target as _;
// use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

extern crate stm32wb_hal as hal;

use core::{fmt::Debug, time::Duration};

use cortex_m_rt::{entry, exception};
use heapless::spsc::{MultiCore, Queue};
use nb::block;

use bbqueue::consts::U514;
use bbqueue::{BBBuffer, ConstBBBuffer};

use hal::{
    flash::FlashExt,
    interrupt,
    prelude::*,
    rcc::{
        ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, RfWakeupClock, RtcClkSrc,
        StopWakeupClock, SysClkSrc,
    },
    tl_mbox::{lhci::LhciC1DeviceInformationCcrp, shci::ShciBleInitCmdParam, TlMbox},
};

use bluetooth_hci::{
    event::{
        command::{CommandComplete, ReturnParameters},
        Event,
    },
    host::{
        uart::{Hci as UartHci, Packet},
        AdvertisingFilterPolicy, EncryptionKey, Hci, OwnAddressType,
    },
    BdAddr, Status,
};

use stm32wb55::{
    event::{command::GattCharacteristicDescriptor, AttributeHandle, Stm32Wb5xEvent},
    gap::{
        AdvertisingDataType, AdvertisingType, Commands as GapCommands, DiscoverableParameters,
        LocalName, Role,
    },
    gatt::{
        AccessPermission, AddCharacteristicParameters, AddDescriptorParameters,
        AddServiceParameters, CharacteristicEvent, CharacteristicHandle, CharacteristicPermission,
        CharacteristicProperty, Commands as GattCommads, DescriptorHandle, DescriptorPermission,
        DescriptorValueParameters, EncryptionKeySize, ServiceHandle, ServiceType,
        UpdateCharacteristicValueParameters, Uuid,
    },
    hal::{Commands as HalCommands, ConfigData, PowerLevel},
    RadioCoprocessor,
};

pub type HciCommandsQueue = Queue<
    fn(&mut RadioCoprocessor<'static, U514>, &BleContext),
    heapless::consts::U32,
    u8,
    MultiCore,
>;

/// Advertisement interval in milliseconds.
const ADV_INTERVAL_MS: u64 = 250;

const BT_NAME: &[u8] = b"hokt";
const BLE_GAP_DEVICE_NAME_LENGTH: u8 = BT_NAME.len() as u8;

#[derive(Debug, Default)]
pub struct BleContext {
    service_handle: Option<ServiceHandle>,
    dev_name_handle: Option<CharacteristicHandle>,
    appearence_handle: Option<CharacteristicHandle>,

    hap_protocol_service_handle: Option<ServiceHandle>,
    hap_protocol_version_handle: Option<CharacteristicHandle>,
    hap_protocol_service_instance_handle: Option<CharacteristicHandle>,

    hap_accessory_information_service_handle: Option<ServiceHandle>,
    hap_accessory_information_identify_handle: Option<CharacteristicHandle>,
}

type RadioCopro = RadioCoprocessor<'static, U514>;

static BB: BBBuffer<U514> = BBBuffer(ConstBBBuffer::new());

static mut RADIO_COPROCESSOR: Option<RadioCopro> = None;

#[entry]
fn entry() -> ! {
    rtt_init_print!(BlockIfFull, 4096);
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
        num_attr_record: 68,
        num_attr_serv: 8,
        attr_value_arr_size: 1344,
        num_of_links: 8,
        extended_packet_length_enable: 1,
        pr_write_list_size: 0x3A,
        mb_lock_count: 0x79,
        att_mtu: 156,
        slave_sca: 500,
        master_sca: 0,
        ls_source: 1,
        max_conn_event_length: 0xFFFFFFFF,
        hs_startup_time: 0x148,
        viterbi_enable: 1,
        ll_only: 0,
        hw_version: 0,
    };

    let (producer, consumer) = BB.try_split().unwrap();
    let rc = RadioCoprocessor::new(producer, consumer, mbox, ipcc, config);

    unsafe {
        RADIO_COPROCESSOR = Some(rc);
    }

    // enable interrupts -> interrupts are enabled in Ipcc::init(), which is called TlMbox::tl_init

    // Boot CPU2
    hal::pwr::set_cpu2(true);

    let ready_event = block!(receive_event());

    rprintln!("Received packet: {:?}", ready_event);

    rprintln!("Resetting processor...");

    let reset_response = perform_command(|rc| rc.reset()).expect("Failed to reset processor");

    rprintln!("Received packet: {:?}", reset_response);

    init_gap_and_gatt().expect("Failed to initialize GAP and GATT");

    rprintln!("Succesfully initialized GAP and GATT");

    init_homekit().expect("Failed to initialize homekit setup");

    rprintln!("Succesfully initialized Homekit");

    loop {
        let response = block!(receive_event());

        rprintln!("Received event: {:x?}", response);
    }
}

fn perform_command(
    command: impl Fn(&mut RadioCopro) -> nb::Result<(), ()>,
) -> Result<ReturnParameters<Stm32Wb5xEvent>, ()> {
    // Send command (blocking)
    block!(cortex_m::interrupt::free(|_| {
        let rc = unsafe { RADIO_COPROCESSOR.as_mut().unwrap() };
        command(rc)
    }))?;

    let response = block!(receive_event()).unwrap(); // .map_err(|_| Err(()))?;

    if let Packet::Event(Event::CommandComplete(CommandComplete {
        return_params,
        num_hci_command_packets: _,
    })) = response
    {
        Ok(return_params)
    } else {
        Err(())
    }
}

fn receive_event() -> nb::Result<
    Packet<Stm32Wb5xEvent>,
    bluetooth_hci::host::uart::Error<(), stm32wb55::event::Stm32Wb5xError>,
> {
    cortex_m::interrupt::free(|_| {
        let rc = unsafe { RADIO_COPROCESSOR.as_mut().unwrap() };
        if rc.process_events() {
            rc.read()
        } else {
            Err(nb::Error::WouldBlock)
        }
    })
}

/// Handles IPCC interrupt and notifies `RadioCoprocessor` code about it.
//#[task(binds = IPCC_C1_RX_IT, resources = [rc])]

// Handle IPCC_C1_RX_IT interrupt
#[interrupt]
fn IPCC_C1_RX_IT() {
    unsafe {
        RADIO_COPROCESSOR.as_mut().unwrap().handle_ipcc_rx();
    }
}

// Handle IPCC_C1_TX_IT interrupt
//
// This interrupt occurs when ??
#[interrupt]
fn IPCC_C1_TX_IT() {
    // TODO: Critical section?
    unsafe {
        RADIO_COPROCESSOR.as_mut().unwrap().handle_ipcc_tx();
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

fn check_status<S: Debug>(status: &Status<S>) -> Result<(), ()> {
    if let Status::Success = status {
        Ok(())
    } else {
        rprintln!("Status not succesfull: {:?}", status);
        Err(())
    }
}

#[derive(Debug)]
struct Service {
    handle: ServiceHandle,

    max_num_attributes: u8,
}

impl Service {
    fn new(service_type: ServiceType, uuid: Uuid, max_attribute_records: u8) -> Result<Self, ()> {
        rprintln!("Adding service {:x?}", uuid);

        let protocol_handle = perform_command(|rc: &mut RadioCopro| {
            let service = AddServiceParameters {
                service_type,
                uuid,
                max_attribute_records,
            };
            rc.add_service(&service)
        })?;

        if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattAddService(
                stm32wb55::event::command::GattService {
                    service_handle,
                    status,
                },
            ),
        ) = protocol_handle
        {
            check_status(&status).expect("Failed to add service");
            rprintln!("Handle {:?}", service_handle);
            Ok(Service {
                handle: service_handle,
                max_num_attributes: max_attribute_records,
            })
        } else {
            //writeln!(serial, "Unexpected response to init_gap command");
            Err(())
        }
    }

    fn add_characteristic(
        &self,
        uuid: &Uuid,
        properties: CharacteristicProperty,
        value_len: usize,
        is_variable: bool,
    ) -> Result<Characteristic, ()> {
        rprintln!("Adding characteristic {:x?}", uuid);
        rprintln!(" Properties: {:?}", properties);

        let response = perform_command(|rc: &mut RadioCopro| {
            rc.add_characteristic(&AddCharacteristicParameters {
                service_handle: self.handle,
                characteristic_uuid: *uuid,
                characteristic_properties: properties,
                characteristic_value_len: value_len,

                is_variable,

                // Initially hardcoded
                gatt_event_mask: CharacteristicEvent::ATTRIBUTE_WRITE,
                encryption_key_size: EncryptionKeySize::with_value(16).unwrap(),
                fw_version_before_v72: false,
                security_permissions: CharacteristicPermission::empty(),
            })
        })?;

        if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattAddCharacteristic(
                stm32wb55::event::command::GattCharacteristic {
                    characteristic_handle,
                    status,
                },
            ),
        ) = response
        {
            check_status(&status).expect("Failed to add characteristic");
            rprintln!("Handle (declaration): {:?}", characteristic_handle);
            rprintln!("Handle (value): {:?}", characteristic_handle.0 + 1);

            // If the notify or indicate properties are set,
            // a CCCD (Client characteristic configuration descriptor) is allocated as well.
            if properties
                .intersects(CharacteristicProperty::NOTIFY | CharacteristicProperty::INDICATE)
            {
                rprintln!(
                    "Client characteristic configuration: Handle={}",
                    characteristic_handle.0 + 1,
                );
            }

            Ok(Characteristic {
                service: self.handle,
                characteristic: characteristic_handle,
                max_len: value_len,
            })
        } else {
            Err(())
        }
    }

    /// Check if this service contains the given handle.
    ///
    /// The check is done based on the maximum number of handles
    /// reserved for this service, as given in the `Service::new`
    /// function. A value of `true` does not guarantee that
    /// the given handle has actually been created, however the
    /// given handle cannot exist in any other service.
    fn contains_handle(&self, handle: AttributeHandle) -> bool {
        let value = handle.0;

        let service_handle = self.handle.0;

        service_handle <= value && value < (service_handle + self.max_num_attributes as u16)
    }
}

struct Characteristic {
    service: ServiceHandle,
    characteristic: CharacteristicHandle,

    max_len: usize,
}

impl Characteristic {
    fn set_value(&self, value: &[u8]) -> Result<(), ()> {
        if value.len() > self.max_len {
            return Err(());
        }

        perform_command(|rc: &mut RadioCopro| {
            rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
                service_handle: self.service,
                characteristic_handle: self.characteristic,
                offset: 0,
                value,
            })
            .map_err(|_| nb::Error::Other(()))
        })?;

        Ok(())
    }

    fn add_descriptor(&self, uuid: Uuid, length: usize) -> Result<DescriptorHandle, ()> {
        let dummy_slice = [0u8; 10];

        assert!(length <= 10, "Hack: Not implemented for length > 10");

        let descriptor = perform_command(|rc: &mut RadioCopro| {
            rc.add_characteristic_descriptor(&mut AddDescriptorParameters {
                service_handle: self.service,
                characteristic_handle: self.characteristic,
                descriptor_uuid: uuid,
                descriptor_value_max_len: length,
                descriptor_value: &dummy_slice[..length],
                security_permissions: DescriptorPermission::empty(),
                access_permissions: AccessPermission::READ,
                gatt_event_mask: CharacteristicEvent::empty(),
                encryption_key_size: EncryptionKeySize::with_value(16).unwrap(),
                is_variable: false,
            })
            .map_err(|_| nb::Error::Other(()))
        })?;

        let descriptor_handle = match descriptor {
            ReturnParameters::Vendor(
                stm32wb55::event::command::ReturnParameters::GattAddCharacteristicDescriptor(
                    GattCharacteristicDescriptor {
                        status,
                        descriptor_handle,
                    },
                ),
            ) => {
                check_status(&status)?;
                descriptor_handle
            }
            _ => {
                // rprintln!( "Unexpected response to init_gap command");
                return Err(());
            }
        };

        rprintln!("Descriptor {:?} - {:?}", uuid, descriptor_handle);

        Ok(descriptor_handle)
    }
}

struct HapService {
    service: Service,
    instance_id: Characteristic,
}

impl HapService {
    fn new(
        service_type: ServiceType,
        uuid: Uuid,
        max_attribute_records: u8,
        instance_id: u16,
    ) -> Result<HapService, ()> {
        let service = Service::new(ServiceType::Primary, uuid, max_attribute_records)?;

        let instance_id_characteristic = service.add_characteristic(
            &Uuid::Uuid128(UUID_SERVICE_INSTANCE),
            CharacteristicProperty::READ,
            2,
            false,
        )?;

        instance_id_characteristic.set_value(&instance_id.to_le_bytes())?;

        Ok(HapService {
            service,
            instance_id: instance_id_characteristic,
        })
    }
}

/// HAP Characteristic
struct HapCharacteristic {
    characteristic: Characteristic,
    characteristic_id: DescriptorHandle,

    max_len: usize,
}

impl HapCharacteristic {
    fn build(
        service: &HapService,
        instance_id: u16,
        uuid: Uuid,
        properties: CharacteristicProperty,
        characteristic_len: usize,
    ) -> Result<Self, ()> {
        let characteristic =
            service
                .service
                .add_characteristic(&uuid, properties, characteristic_len, false)?;

        let descriptor_handle =
            characteristic.add_descriptor(Uuid::Uuid128(UUID_CHARACTERISTIC_ID), 2)?;

        //rprintln!( "Descriptor handle: {:?}", descriptor_handle);

        let response = perform_command(|rc| {
            rc.set_descriptor_value(&DescriptorValueParameters {
                service_handle: characteristic.service,
                characteristic_handle: characteristic.characteristic,
                descriptor_handle,
                offset: 0,
                value: &instance_id.to_le_bytes(),
            })
            .map_err(|_| nb::Error::Other(()))
        })?;

        // rprintln!(
        //     serial,
        //     "Response to setting descriptor value: {:?}",
        //     response
        // );

        Ok(HapCharacteristic {
            characteristic,
            characteristic_id: descriptor_handle,
            max_len: characteristic_len,
        })
    }

    fn set_value(&self, value: &[u8]) -> Result<(), ()> {
        rprintln!(
            "{:?}: value={:x?}",
            self.characteristic.characteristic,
            value
        );
        self.characteristic.set_value(value)
    }
}

fn init_gap_and_gatt() -> Result<(), ()> {
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

    let mut ble_context = BleContext::default();

    let return_params =
        perform_command(|rc| rc.init_gap(Role::PERIPHERAL, false, BLE_GAP_DEVICE_NAME_LENGTH))?;

    if let ReturnParameters::Vendor(stm32wb55::event::command::ReturnParameters::GapInit(
        stm32wb55::event::command::GapInit {
            service_handle,
            dev_name_handle,
            appearance_handle,
            ..
        },
    )) = return_params
    {
        ble_context.service_handle = Some(service_handle);
        ble_context.dev_name_handle = Some(dev_name_handle);
        ble_context.appearence_handle = Some(appearance_handle);
    } else {
        rprintln!("Unexpected response to init_gap command");
        return Err(());
    }

    perform_command(|rc| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle: ble_context.service_handle.unwrap(),
            characteristic_handle: ble_context.dev_name_handle.unwrap(),
            offset: 0,
            value: BT_NAME,
        })
        .map_err(|_| nb::Error::Other(()))
    })?;

    // hci_commands_queue
    //     .enqueue(|rc, cx| {
    //         rc.add_characteristic(&AddCharacteristicParameters {
    //             service_handle: cx
    //                 .hap_protocol_service_handle
    //                 .expect("service handle to be set"),
    //             characteristic_uuid: Uuid::Uuid128(UUID_PROTOCOL_SIGNATURE),
    //             //characteristic_value: b"2.2.0",
    //             characteristic_value_len: 64,
    //             security_permissions: CharacteristicPermission::empty(),
    //             //access_permissions: AccessPermission::READ,
    //             characteristic_properties: CharacteristicProperty::READ,
    //             gatt_event_mask: CharacteristicEvent::empty(),
    //             encryption_key_size: EncryptionKeySize::with_value(16).unwrap(),
    //             is_variable: false,
    //             fw_version_before_v72: false,
    //         })
    //         .unwrap()
    //     })
    //     .ok();

    // Acessory information service
    rprintln!("Accessory information service");

    //cx.next_service = BleServices::AccessoryInformation;
    let accessory_service = HapService::new(
        ServiceType::Primary,
        Uuid::Uuid128(UUID_ACCESSORY_INFORMATION),
        30,
        1,
    )?;

    // add the

    let _information_identify_characteristic = HapCharacteristic::build(
        &accessory_service,
        2,
        Uuid::Uuid128(UUID_ACCESSORY_INFORMATION_IDENTIFY),
        CharacteristicProperty::WRITE,
        1,
    )?;

    let information_manufacturer_characteristic = HapCharacteristic::build(
        &accessory_service,
        3,
        Uuid::Uuid128(UUID_ACCESSORY_INFORMATION_MANUFACTURER),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        64,
    )?;
    information_manufacturer_characteristic.set_value(b"Dominik Corp.\0")?;

    let information_model_characteristic = HapCharacteristic::build(
        &accessory_service,
        4,
        Uuid::Uuid128(UUID_ACCESSORY_INFORMATION_MODEL),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        10,
    )?;
    information_model_characteristic.set_value(b"M001\0")?;

    let information_name_characteristic = HapCharacteristic::build(
        &accessory_service,
        5,
        Uuid::Uuid128(UUID_ACCESSORY_INFORMATION_NAME),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        10,
    )?;
    information_name_characteristic.set_value(BT_NAME)?;

    let information_serial_number_characteristic = HapCharacteristic::build(
        &accessory_service,
        6,
        Uuid::Uuid128(UUID_ACCESSORY_INFORMATION_SERIAL_NUMBER),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        15,
    )?;
    information_serial_number_characteristic.set_value(b"S12345\0")?;

    let information_firmware_revision_characteristic = HapCharacteristic::build(
        &accessory_service,
        7,
        Uuid::Uuid128(UUID_ACCESSORY_INFORMATION_FIRMWARE_REVISION),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        10,
    )?;
    information_firmware_revision_characteristic.set_value(b"1.0.0\0")?;

    let information_hardware_revision_characteristic = HapCharacteristic::build(
        &accessory_service,
        8,
        Uuid::Uuid128(UUID_ACCESSORY_INFORMATION_HARDWARE_REVISION),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        10,
    )?;
    information_hardware_revision_characteristic.set_value(b"1.0.0\0")?;

    // Protocol information service

    rprintln!("Protocol information service");

    let protocol_information_service = HapService::new(
        ServiceType::Primary,
        Uuid::Uuid128(UUID_PROTOCOL_INFORMATION),
        10,
        0x10,
    )?;

    let _service_signature_characteristic = HapCharacteristic::build(
        &protocol_information_service,
        0x11,
        Uuid::Uuid128(UUID_SERVICE_SIGNATURE),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        100,
    )?;

    // Indicate that the protocol service support configuration (7.4.3, p. 121, HAP Specification)
    //service_signature_characteristic.set_value(&[0x04, 0x00])?;

    let protocol_version_characteristic = HapCharacteristic::build(
        &protocol_information_service,
        0x12,
        Uuid::Uuid128(UUID_VERSION_CHARACTERISTIC),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        64,
    )?;

    protocol_version_characteristic.set_value(b"2.2.0\0")?;

    // Add Pairing service
    rprintln!("Pairing service");
    let pairing_service = HapService::new(
        ServiceType::Primary,
        Uuid::Uuid128(UUID_PAIRING_SERVICE),
        20,
        0x20,
    )?;

    let pair_setup = HapCharacteristic::build(
        &pairing_service,
        0x22,
        Uuid::Uuid128(UUID_PAIRING_SETUP),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        1,
    )?;

    let pair_verify = HapCharacteristic::build(
        &pairing_service,
        0x23,
        Uuid::Uuid128(UUID_PAIRING_VERIFY),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        1,
    )?;
    let pairing_features = HapCharacteristic::build(
        &pairing_service,
        0x24,
        Uuid::Uuid128(UUID_PAIRING_FEATURES),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        1,
    )?;
    let pairing_pairings = HapCharacteristic::build(
        &pairing_service,
        0x25,
        Uuid::Uuid128(UUID_PAIRING_PAIRINGS),
        CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        1,
    )?;

    Ok(())
}

/// Build a Homekit UUID from the first four  bytes of the UUID
const fn apple_uuid(data: u32) -> [u8; 16] {
    let mut common: [u8; 16] = [
        0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x3E, 0x00, 0x00,
        0x00,
    ];

    // The UUID is stored in ?little-endian? format, so we have to change the order of the data
    common[12] = (data & 0xff) as u8;
    common[13] = ((data >> 8) & 0xff) as u8;
    common[14] = ((data >> 16) & 0xff) as u8;
    common[15] = ((data >> 24) & 0xff) as u8;

    common
}

// UUID for Identify characteristic
// public.hap.characteristic.identify
const UUID_ACCESSORY_INFORMATION: [u8; 16] = apple_uuid(0x3E);

// public.hap.characteristic.firmware.revision (9.40, p. 177)
const UUID_ACCESSORY_INFORMATION_FIRMWARE_REVISION: [u8; 16] = apple_uuid(0x52);

// public.hap.characteristic.hardware.revision (9.41, p. 178)
const UUID_ACCESSORY_INFORMATION_HARDWARE_REVISION: [u8; 16] = apple_uuid(0x53);

// public.hap.characteristic.identify (9.45, p. 180)
const UUID_ACCESSORY_INFORMATION_IDENTIFY: [u8; 16] = apple_uuid(0x14);

/// public.hap.characteristic.manufacturer (9.58, p. 187)
const UUID_ACCESSORY_INFORMATION_MANUFACTURER: [u8; 16] = apple_uuid(0x20);

/// public.hap.characteristic.model (9.59, p. 187)
const UUID_ACCESSORY_INFORMATION_MODEL: [u8; 16] = apple_uuid(0x21);

/// public.hap.characteristic.name (9.62, p. 188)
const UUID_ACCESSORY_INFORMATION_NAME: [u8; 16] = apple_uuid(0x23);

// public.hap.characteristic.firmware.revision (9.87, p. 201)
const UUID_ACCESSORY_INFORMATION_SERIAL_NUMBER: [u8; 16] = apple_uuid(0x30);

// UUID for HAP Protocol Information service
// public.hap.service.protocol.information.service
const UUID_PROTOCOL_INFORMATION: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xA2, 0x00, 0x00, 0x00,
];

const _UUID_PROTOCOL_SIGNATURE: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xA5, 0x00, 0x00, 0x00,
];

const UUID_PAIRING_SERVICE: [u8; 16] = apple_uuid(0x55);

const UUID_PAIRING_SETUP: [u8; 16] = apple_uuid(0x4C);
const UUID_PAIRING_VERIFY: [u8; 16] = apple_uuid(0x4E);

const UUID_PAIRING_FEATURES: [u8; 16] = apple_uuid(0x4F);
const UUID_PAIRING_PAIRINGS: [u8; 16] = apple_uuid(0x50);

/// UUID for the Service Instance ID.
///
/// Every HAP service must have a readonly service instance ID iwth this UUID.
const UUID_SERVICE_INSTANCE: [u8; 16] = [
    0xD1, 0xA0, 0x83, 0x50, 0x00, 0xAA, 0xD3, 0x87, 0x17, 0x48, 0x59, 0xA7, 0x5D, 0xE9, 0x04, 0xE6,
];

// UUID
const UUID_VERSION_CHARACTERISTIC: [u8; 16] = [
    // "00000037-0000-1000-8000-0026BB765291"
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00,
];

const UUID_CHARACTERISTIC_ID: [u8; 16] = [
    // DC46F0FE-81D2-4616-B5D9-6A BD D7 96 93 9A
    0x9A, 0x93, 0x96, 0xD7, 0xBD, 0x6A, 0xD9, 0xB5, 0x16, 0x46, 0xD2, 0x81, 0xFE, 0xF0, 0x46, 0xDC,
];

/// Service Signature HAP characteristic
const UUID_SERVICE_SIGNATURE: [u8; 16] = apple_uuid(0xA5);

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

fn init_homekit() -> Result<(), ()> {
    // Disable scan response
    perform_command(|rc: &mut RadioCopro| {
        rc.le_set_scan_response_data(&[])
            .map_err(|_| nb::Error::Other(()))
    })?;

    // Put the device in a non-connectable mode
    perform_command(|rc| {
        let params = DiscoverableParameters {
            advertising_type: AdvertisingType::ConnectableUndirected,
            advertising_interval: Some((
                Duration::from_millis(ADV_INTERVAL_MS),
                Duration::from_millis(ADV_INTERVAL_MS),
            )),
            address_type: OwnAddressType::Public,
            filter_policy: AdvertisingFilterPolicy::AllowConnectionAndScan,
            // Local name should be empty for the device to be recognized as an Eddystone beacon
            local_name: Some(LocalName::Complete(BT_NAME)),
            advertising_data: &[],
            conn_interval: (None, None),
        };

        rc.set_discoverable(&params)
            .map_err(|_| nb::Error::Other(()))
    })?;

    perform_command(|rc| {
        let advertising_data = [
            0x12, // Length
            0xff, // Manufacturer Data
            0x4c, 0x00, // Apple ID
            0x06, // Type
            0x2D, // STL
            0x01, // SF
            0x44, 0x55, 0x66, 0x44, 0x55, 0x66, // Device ID
            0x00, 0x0A, // ACID G
            0x00, 0x01, // GSN
            0x2,  // Configuration number
            0x2,  // CV
                  //0x00, 0x00, 0x00, 0x00, // Secure Hash,
        ];

        rc.update_advertising_data(&advertising_data[..])
            .map_err(|_| nb::Error::Other(()))
    })?;

    perform_command(|rc| {
        let mut service_uuid_list = [0u8; 16 * 1 + 2];

        service_uuid_list[0] = 16 * 1 + 1;
        service_uuid_list[1] = AdvertisingDataType::Uuid128 as u8;

        for i in 0..16 {
            service_uuid_list[i + 2] = UUID_PAIRING_SERVICE[i];
        }

        rc.update_advertising_data(&service_uuid_list[..])
            .map_err(|_| nb::Error::Other(()))
    })?;

    perform_command(|rc| {
        let flags = [2, AdvertisingDataType::Flags as u8, 0x4 | 0x2];

        rc.update_advertising_data(&flags[..])
            .map_err(|_| nb::Error::Other(()))
    })?;

    Ok(())
}
