//! BLE Eddystone URL beacon example.
#![no_main]
#![no_std]
#![allow(non_snake_case)]

extern crate panic_reset;
extern crate stm32wb_hal as hal;

use core::{fmt::Write, time::Duration};

use cortex_m_rt::{entry, exception};
use heapless::spsc::{MultiCore, Queue};
use nb::block;

use hal::{
    flash::FlashExt,
    interrupt,
    prelude::*,
    rcc::{
        ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, RfWakeupClock, RtcClkSrc,
        StopWakeupClock, SysClkSrc,
    },
    serial::Serial,
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
    BdAddr,
};

use stm32wb55::{
    event::Stm32Wb5xEvent,
    gap::{
        AdvertisingDataType, AdvertisingType, Commands as GapCommands, DiscoverableParameters,
        LocalName, Role,
    },
    gatt::{
        AddCharacteristicParameters, AddServiceParameters, CharacteristicEvent,
        CharacteristicHandle, CharacteristicPermission, CharacteristicProperty,
        Commands as GattCommads, EncryptionKeySize, ServiceHandle, ServiceType,
        UpdateCharacteristicValueParameters, Uuid,
    },
    hal::{Commands as HalCommands, ConfigData, PowerLevel},
    RadioCoprocessor,
};

pub type HciCommandsQueue =
    Queue<fn(&mut RadioCoprocessor<'static>, &BleContext), heapless::consts::U32, u8, MultiCore>;

/// Advertisement interval in milliseconds.
const ADV_INTERVAL_MS: u64 = 250;

// Need to be at least 257 bytes to hold biggest possible HCI BLE event + header
const BLE_DATA_BUF_SIZE: usize = 257 * 2;
const BLE_GAP_DEVICE_NAME_LENGTH: u8 = 7;

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

static mut RADIO_COPROCESSOR: Option<RadioCoprocessor> = None;

#[entry]
fn entry() -> ! {
    run();

    loop {
        continue;
    }
}

static mut BLE_DATA_BUF: [u8; BLE_DATA_BUF_SIZE] = [0u8; BLE_DATA_BUF_SIZE];

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

    let mut gpiob = dp.GPIOB.split(&mut rcc);

    let tx = gpiob
        .pb6
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
        .into_af7(&mut gpiob.moder, &mut gpiob.afrl);
    let rx = gpiob.pb7.into_af7(&mut gpiob.moder, &mut gpiob.afrl);

    let mut serial = Serial::usart1(dp.USART1, (tx, rx), 115_200.bps(), &mut rcc);

    let _ = writeln!(serial, "Boot");

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
    let rc = unsafe { RadioCoprocessor::new(&mut BLE_DATA_BUF[..], mbox, ipcc, config) };

    unsafe {
        RADIO_COPROCESSOR = Some(rc);
    }

    // enable interrupts -> interrupts are enabled in Ipcc::init(), which is called TlMbox::tl_init

    // Boot CPU2
    hal::pwr::set_cpu2(true);

    let ready_event = block!(receive_event());

    let _ = writeln!(serial, "Received packet: {:?}", ready_event);

    let _ = writeln!(serial, "Resetting processor...");

    let reset_response = perform_command(|rc| rc.reset()).expect("Failed to reset processor");

    let _ = writeln!(serial, "Received packet: {:?}", reset_response);

    init_gap_and_gatt(&mut serial).expect("Failed to initialize GAP and GATT");

    let _ = writeln!(serial, "Succesfully initialized GAP and GATT");

    init_homekit().expect("Failed to initialize homekit setup");

    let _ = writeln!(serial, "Succesfully initialized Homekit");
}

fn perform_command(
    command: impl Fn(&mut RadioCoprocessor<'static>) -> nb::Result<(), ()>,
) -> Result<ReturnParameters<Stm32Wb5xEvent>, ()> {
    // Send command (blocking)
    block!(cortex_m::interrupt::free(|_| {
        let rc = unsafe { RADIO_COPROCESSOR.as_mut().unwrap() };
        command(rc)
    }))?;

    let response = block!(receive_event())?;

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

fn receive_event() -> nb::Result<Packet<Stm32Wb5xEvent>, ()> {
    cortex_m::interrupt::free(|_| {
        let rc = unsafe { RADIO_COPROCESSOR.as_mut().unwrap() };
        if rc.process_events() {
            match rc.read() {
                Ok(event) => Ok(event),
                Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
                Err(nb::Error::Other(_)) => Err(nb::Error::Other(())),
            }
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

#[derive(Debug)]
struct Service(ServiceHandle);

impl Service {
    fn new(
        service_type: ServiceType,
        uuid: Uuid,
        max_attribute_records: usize,
    ) -> Result<Self, ()> {
        let protocol_handle = perform_command(|rc| {
            let service = AddServiceParameters {
                service_type,
                uuid,
                max_attribute_records,
            };
            rc.add_service(&service)
        })?;

        if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattAddService(
                stm32wb55::event::command::GattService { service_handle, .. },
            ),
        ) = protocol_handle
        {
            Ok(Service(service_handle))
        } else {
            //writeln!(serial, "Unexpected response to init_gap command");
            Err(())
        }
    }

    fn add_characteristic(
        &self,
        characteristic: &mut AddCharacteristicParameters,
    ) -> Result<Characteristic, ()> {
        // TODO: Better way of implementing this
        characteristic.service_handle = self.0;

        let response = perform_command(|rc| rc.add_characteristic(characteristic))?;

        if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattAddCharacteristic(
                stm32wb55::event::command::GattCharacteristic {
                    characteristic_handle,
                    ..
                },
            ),
        ) = response
        {
            Ok(Characteristic {
                service: self.0,
                characteristic: characteristic_handle,
            })
        } else {
            Err(())
        }
    }
}

struct Characteristic {
    service: ServiceHandle,
    characteristic: CharacteristicHandle,
}

fn init_gap_and_gatt(serial: &mut impl Write) -> Result<(), ()> {
    let response = perform_command(|rc| {
        rc.write_config_data(&ConfigData::public_address(get_bd_addr()).build())
    })?;

    let _ = writeln!(serial, "Response to write_config_data: {:?}", response);

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
        let _ = writeln!(serial, "Unexpected response to init_gap command");
        return Err(());
    }

    perform_command(|rc| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle: ble_context.service_handle.unwrap(),
            characteristic_handle: ble_context.dev_name_handle.unwrap(),
            offset: 0,
            value: b"hokt",
        })
        .map_err(|_| nb::Error::Other(()))
    })?;

    // Protocol information service

    //cx.resources.ble.next_service = BleServices::Protocol;eh

    let protocol_information_service = Service::new(
        ServiceType::Secondary,
        Uuid::Uuid128(UUID_PROTOCOL_INFORMATION),
        20,
    )?;

    let _ = writeln!(serial, "Added service: {:?}", protocol_information_service);

    let _protocol_version_characteristic =
        protocol_information_service.add_characteristic(&mut AddCharacteristicParameters {
            service_handle: ServiceHandle(0), // Overwritten by add_characteristic
            characteristic_uuid: Uuid::Uuid128(UUID_VERSION_CHARACTERISTIC),
            //characteristic_value: b"2.2.0",
            characteristic_value_len: 6,
            security_permissions: CharacteristicPermission::empty(),
            //access_permissions: AccessPermission::READ,
            characteristic_properties: CharacteristicProperty::READ | CharacteristicProperty::WRITE,
            gatt_event_mask: CharacteristicEvent::empty(),
            encryption_key_size: EncryptionKeySize::with_value(16).unwrap(),
            is_variable: false,
            fw_version_before_v72: false,
        })?;

    // hci_commands_queue
    //     .enqueue(|rc, cx| {
    //         rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
    //             service_handle: cx.hap_protocol_service_handle.unwrap(),
    //             characteristic_handle: cx.hap_protocol_version_handle.unwrap(),
    //             offset: 0,
    //             value: b"2.2.0",
    //         })
    //         .unwrap()
    //     })
    //     .ok();

    let service_instance_characteristic =
        protocol_information_service.add_characteristic(&mut AddCharacteristicParameters {
            service_handle: ServiceHandle(0), // Overwritten by add_characteristic
            characteristic_uuid: Uuid::Uuid128(UUID_SERVICE_INSTANCE),
            //characteristic_value: b"2.2.0",
            characteristic_value_len: 2,
            security_permissions: CharacteristicPermission::empty(),
            //access_permissions: AccessPermission::READ,
            characteristic_properties: CharacteristicProperty::READ,
            gatt_event_mask: CharacteristicEvent::empty(),
            encryption_key_size: EncryptionKeySize::with_value(16).unwrap(),
            is_variable: false,
            fw_version_before_v72: false,
        })?;

    perform_command(|rc| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle: service_instance_characteristic.service,
            characteristic_handle: service_instance_characteristic.characteristic,
            offset: 0,
            value: &[0x00, 0x10],
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

    //cx.next_service = BleServices::AccessoryInformation;
    let accessory_service = Service::new(
        ServiceType::Primary,
        Uuid::Uuid128(UUID_ACCESSORY_INFORMATION),
        20,
    )?;

    let _information_identify_characteristic =
        accessory_service.add_characteristic(&mut AddCharacteristicParameters {
            service_handle: ServiceHandle(0),
            characteristic_uuid: Uuid::Uuid128(UUID_ACCESSORY_INFORMATION_IDENTIFY),
            //characteristic_value: b"2.2.0",
            characteristic_value_len: 2,
            security_permissions: CharacteristicPermission::empty(),
            characteristic_properties: CharacteristicProperty::WRITE,
            gatt_event_mask: CharacteristicEvent::ATTRIBUTE_WRITE
                | CharacteristicEvent::CONFIRM_WRITE,
            encryption_key_size: EncryptionKeySize::with_value(16).unwrap(),
            is_variable: false,
            fw_version_before_v72: false,
        })?;

    // Pairing service
    // hci_commands_queue
    //     .enqueue(|rc, _| {
    //         let service = AddServiceParameters {
    //             service_type: ServiceType::Primary,
    //             uuid: Uuid::Uuid128(UUID_PAIRING_SERVICE),
    //             max_attribute_records: 5,
    //         };
    //         rc.add_service(&service).expect("Adding service")
    //     })
    //     .ok();

    Ok(())
}

const UUID_ACCESSORY_INFORMATION: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00,
];

const UUID_ACCESSORY_INFORMATION_IDENTIFY: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
];

const UUID_PROTOCOL_INFORMATION: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xA2, 0x00, 0x00, 0x00,
];

const _UUID_PROTOCOL_SIGNATURE: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xA5, 0x00, 0x00, 0x00,
];

const UUID_PAIRING_SERVICE: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x55, 0x00, 0x00, 0x00,
];

const _UUID_PAIRING_SETUP: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x4C, 0x00, 0x00, 0x00,
];
const _UUID_PAIRING_FEATURES: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x4E, 0x00, 0x00, 0x00,
];

const _UUID_PAIRING_PARIRINGS: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x4F, 0x00, 0x00, 0x00,
];

const UUID_SERVICE_INSTANCE: [u8; 16] = [
    0xD1, 0xA0, 0x83, 0x50, 0x00, 0xAA, 0xD3, 0x87, 0x17, 0x48, 0x59, 0xA7, 0x5D, 0xE9, 0x04, 0xE6,
];

const UUID_VERSION_CHARACTERISTIC: [u8; 16] = [
    // "00000037-0000-1000-8000-0026BB765291"
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00,
];

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
    perform_command(|rc| {
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
            local_name: Some(LocalName::Complete(b"hokt")),
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
            0x1,  // Configuration number
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
