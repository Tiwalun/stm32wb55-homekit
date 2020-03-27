//! BLE Eddystone URL beacon example.
#![no_main]
#![no_std]
#![allow(non_snake_case)]

extern crate panic_reset;
extern crate stm32wb_hal as hal;

use core::{fmt::Write, time::Duration};

use cortex_m_rt::exception;
use heapless::spsc::{MultiCore, Queue};
use nb::block;
use rtfm::app;

use hal::{
    flash::FlashExt,
    gpio::{
        gpiob::{PB6, PB7},
        Alternate, Input, Output, AF7,
    },
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
    gap::{
        AdvertisingDataType, AdvertisingType, Commands as GapCommands, DiscoverableParameters,
        LocalName, Role,
    },
    gatt::{
        AccessPermission, AddCharacteristicParameters, AddDescriptorParameters,
        AddServiceParameters, CharacteristicEvent, CharacteristicHandle, CharacteristicPermission,
        CharacteristicProperty, Commands as GattCommads, DescriptorPermission, EncryptionKeySize,
        ServiceHandle, ServiceType, UpdateCharacteristicValueParameters, Uuid,
    },
    hal::{Commands as HalCommands, ConfigData, PowerLevel},
    RadioCoprocessor,
};
use stm32wb_pac::USART1;

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

    next_service: BleServices,
}

#[derive(Debug)]
enum BleServices {
    Protocol,
    AccessoryInformation,
    NoneExpected,
}

impl Default for BleServices {
    fn default() -> Self {
        BleServices::NoneExpected
    }
}

#[app(device = stm32wb_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources<USART, TXPIN, RXPIN> {
        rc: RadioCoprocessor<'static>,
        hci_commands_queue: HciCommandsQueue,
        ble_context: BleContext,
        serial: Serial<
            USART1,
            PB6<Alternate<AF7, Output<hal::gpio::PushPull>>>,
            PB7<Alternate<AF7, Input<hal::gpio::Floating>>>,
        >,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut BLE_DATA_BUF: [u8; BLE_DATA_BUF_SIZE] = [0u8; BLE_DATA_BUF_SIZE];

        let dp = cx.device;
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

        write!(serial, "Bootup 10\r\n").unwrap();

        // RTC is required for proper operation of BLE stack
        let _rtc = hal::rtc::Rtc::rtc(dp.RTC, &mut rcc);

        let mut ipcc = dp.IPCC.constrain();
        let mbox = TlMbox::tl_init(&mut rcc, &mut ipcc);

        // Boot CPU2
        hal::pwr::set_cpu2(true);

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
        let rc = RadioCoprocessor::new(&mut BLE_DATA_BUF[..], mbox, ipcc, config);

        init::LateResources {
            rc,
            hci_commands_queue: HciCommandsQueue::u8(),
            ble_context: BleContext::default(),
            serial,
        }
    }

    #[idle(resources = [rc, ble_context], spawn = [setup, exec_hci, event])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();

            // At this point, an interrupt was received.
            // Radio co-processor talks to the app via IPCC interrupts, so this interrupt
            // may be one of the IPCC interrupts and the app can start processing events from
            // radio co-processor here.
            let evt = cx.resources.rc.lock(|rc| {
                if rc.process_events() {
                    Some(block!(rc.read()))
                } else {
                    None
                }
            });

            if let Some(Ok(Packet::Event(evt))) = evt {
                if let Event::Vendor(stm32wb55::event::Stm32Wb5xEvent::CoprocessorReady(_)) = evt {
                    // Setup BLE service when BLE co-processor is ready
                    cx.spawn.setup().unwrap();
                } else {
                    cx.spawn.event(evt).unwrap();
                    cx.spawn.exec_hci().unwrap();
                }
            }
        }
    }

    /// Sets up Eddystone BLE beacon service.
    #[task(resources = [rc, hci_commands_queue], spawn = [exec_hci])]
    fn setup(mut cx: setup::Context) {
        cx.resources
            .hci_commands_queue
            .enqueue(|rc, _| rc.reset().unwrap())
            .ok();

        init_gap_and_gatt(&mut cx.resources.hci_commands_queue);
        init_eddystone(&mut cx.resources.hci_commands_queue);

        // Execute first HCI command from the queue
        cx.spawn.exec_hci().unwrap();
    }

    /// Executes HCI command from the queue.
    #[task(resources = [rc, hci_commands_queue, ble_context])]
    fn exec_hci(mut cx: exec_hci::Context) {
        if let Some(cmd) = cx.resources.hci_commands_queue.dequeue() {
            cmd(&mut cx.resources.rc, &cx.resources.ble_context);
        }
    }

    /// Processes BLE events.
    #[task(resources = [ble_context, serial])]
    fn event(mut cx: event::Context, event: Event<stm32wb55::event::Stm32Wb5xEvent>) {
        write!(cx.resources.serial, "{:?}\r\n", &event);

        if let Event::CommandComplete(CommandComplete { return_params, .. }) = event {
            match return_params {
                ReturnParameters::Vendor(stm32wb55::event::command::ReturnParameters::GapInit(
                    stm32wb55::event::command::GapInit {
                        service_handle,
                        dev_name_handle,
                        appearance_handle,
                        ..
                    },
                )) => {
                    cx.resources.ble_context.service_handle = Some(service_handle);
                    cx.resources.ble_context.dev_name_handle = Some(dev_name_handle);
                    cx.resources.ble_context.appearence_handle = Some(appearance_handle);
                }
                ReturnParameters::Vendor(
                    stm32wb55::event::command::ReturnParameters::GattAddService(
                        stm32wb55::event::command::GattService { service_handle, .. },
                    ),
                ) => {
                    if cx
                        .resources
                        .ble_context
                        .hap_protocol_service_handle
                        .is_none()
                    {
                        cx.resources.ble_context.hap_protocol_service_handle = Some(service_handle);
                    } else {
                        cx.resources
                            .ble_context
                            .hap_accessory_information_service_handle = Some(service_handle);
                    }
                }
                ReturnParameters::Vendor(
                    stm32wb55::event::command::ReturnParameters::GattAddCharacteristic(
                        stm32wb55::event::command::GattCharacteristic {
                            characteristic_handle,
                            ..
                        },
                    ),
                ) => {
                    if cx
                        .resources
                        .ble_context
                        .hap_protocol_version_handle
                        .is_none()
                    {
                        cx.resources.ble_context.hap_protocol_version_handle =
                            Some(characteristic_handle);
                    } else if cx
                        .resources
                        .ble_context
                        .hap_protocol_service_instance_handle
                        .is_none()
                    {
                        cx.resources
                            .ble_context
                            .hap_protocol_service_instance_handle = Some(characteristic_handle);
                    } else {
                        cx.resources
                            .ble_context
                            .hap_accessory_information_identify_handle =
                            Some(characteristic_handle);
                    }
                }

                _ => (),
            }
        }
    }

    /// Handles IPCC interrupt and notifies `RadioCoprocessor` code about it.
    #[task(binds = IPCC_C1_RX_IT, resources = [rc])]
    fn mbox_rx(cx: mbox_rx::Context) {
        cx.resources.rc.handle_ipcc_rx();
    }

    /// Handles IPCC interrupt and notifies `RadioCoprocessor` code about it.
    #[task(binds = IPCC_C1_TX_IT, resources = [rc])]
    fn mbox_tx(cx: mbox_tx::Context) {
        cx.resources.rc.handle_ipcc_tx();
    }

    // Interrupt handlers used to dispatch software tasks.
    // One per priority.
    extern "C" {
        fn LPUART1();
    }

    //#[task(binds = USART1)]
    //fn usart_handler() {}
};

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

fn init_gap_and_gatt(hci_commands_queue: &mut HciCommandsQueue) {
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::public_address(get_bd_addr()).build())
                .expect("set public address");
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::random_address(get_random_addr()).build())
                .expect("set random address");
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::identity_root(&get_irk()).build())
                .expect("set IRK address");
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::encryption_root(&get_erk()).build())
                .expect("set ERK address");
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.set_tx_power_level(PowerLevel::ZerodBm)
                .expect("set TX power level")
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| rc.init_gatt().expect("GATT init"))
        .ok();

    hci_commands_queue
        .enqueue(|rc, _| {
            rc.init_gap(Role::PERIPHERAL, false, BLE_GAP_DEVICE_NAME_LENGTH)
                .expect("GAP init")
        })
        .ok();

    hci_commands_queue
        .enqueue(|rc, cx| {
            rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
                service_handle: cx.service_handle.expect("service handle to be set"),
                characteristic_handle: cx.dev_name_handle.expect("dev name handle to be set"),
                offset: 0,
                value: b"hokt",
            })
            .unwrap()
        })
        .ok();

    // Protocol information service

    //cx.resources.ble.next_service = BleServices::Protocol;

    hci_commands_queue
        .enqueue(|rc, _| {
            let service = AddServiceParameters {
                service_type: ServiceType::Secondary,
                uuid: Uuid::Uuid128(UUID_PROTOCOL_INFORMATION),
                max_attribute_records: 20,
            };
            rc.add_service(&service).expect("Adding service")
        })
        .ok();

    hci_commands_queue
        .enqueue(|rc, cx| {
            rc.add_characteristic(&AddCharacteristicParameters {
                service_handle: cx
                    .hap_protocol_service_handle
                    .expect("service handle to be set"),
                characteristic_uuid: Uuid::Uuid128(UUID_VERSION_CHARACTERISTIC),
                //characteristic_value: b"2.2.0",
                characteristic_value_len: 6,
                security_permissions: CharacteristicPermission::empty(),
                //access_permissions: AccessPermission::READ,
                characteristic_properties: CharacteristicProperty::READ
                    | CharacteristicProperty::WRITE,
                gatt_event_mask: CharacteristicEvent::empty(),
                encryption_key_size: EncryptionKeySize::with_value(16).unwrap(),
                is_variable: false,
                fw_version_before_v72: false,
            })
            .unwrap()
        })
        .ok();

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

    hci_commands_queue
        .enqueue(|rc, cx| {
            rc.add_characteristic(&AddCharacteristicParameters {
                service_handle: cx
                    .hap_protocol_service_handle
                    .expect("service handle to be set"),
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
            })
            .unwrap()
        })
        .ok();

    hci_commands_queue
        .enqueue(|rc, cx| {
            rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
                service_handle: cx.hap_protocol_service_handle.unwrap(),
                characteristic_handle: cx.hap_protocol_service_instance_handle.unwrap(),
                offset: 0,
                value: &[0x00, 0x10],
            })
            .unwrap()
        })
        .ok();

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
    hci_commands_queue
        .enqueue(|rc, _| {
            let service = AddServiceParameters {
                service_type: ServiceType::Primary,
                uuid: Uuid::Uuid128(UUID_ACCESSORY_INFORMATION),
                max_attribute_records: 20,
            };
            rc.add_service(&service).expect("Adding service")
        })
        .ok();

    hci_commands_queue
        .enqueue(|rc, cx| {
            rc.add_characteristic(&AddCharacteristicParameters {
                service_handle: cx
                    .hap_accessory_information_service_handle
                    .expect("service handle to be set"),
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
            })
            .unwrap()
        })
        .ok();

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

const UUID_PROTOCOL_SIGNATURE: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xA5, 0x00, 0x00, 0x00,
];

const UUID_PAIRING_SERVICE: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x55, 0x00, 0x00, 0x00,
];

const UUID_PAIRING_SETUP: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x4C, 0x00, 0x00, 0x00,
];
const UUID_PAIRING_FEATURES: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x4E, 0x00, 0x00, 0x00,
];

const UUID_PAIRING_PARIRINGS: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x4F, 0x00, 0x00, 0x00,
];

const UUID_SERVICE_INSTANCE: [u8; 16] = [
    0xD1, 0xA0, 0x83, 0x50, 0x00, 0xAA, 0xD3, 0x87, 0x17, 0x48, 0x59, 0xA7, 0x5D, 0xE9, 0x04, 0xE6,
];

const UUID_VERSION_CHARACTERISTIC: [u8; 16] = [
    // "00000037-0000-1000-8000-0026BB765291"
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00,
];

#[derive(Copy, Clone)]
#[allow(dead_code)]
enum EddystoneUrlScheme {
    HttpWww = 0x00,
    HttpsWww = 0x01,
    Http = 0x02,
    Https = 0x03,
}

fn init_eddystone(hci_commands_queue: &mut HciCommandsQueue) {
    // Disable scan response
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.le_set_scan_response_data(&[])
                .expect("set scan response data")
        })
        .ok();

    // Put the device in a non-connectable mode
    hci_commands_queue
        .enqueue(|rc, _| {
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
                .expect("set discoverable params")
        })
        .ok();

    // Remove some advertisements (this is done to decrease the packet size)
    //hci_commands_queue
    //    .enqueue(|rc, _| {
    //        rc.delete_ad_type(AdvertisingDataType::TxPowerLevel)
    //            .expect("delete tx power ad type")
    //    })
    //    .ok();
    //hci_commands_queue
    //    .enqueue(|rc, _| {
    //        rc.delete_ad_type(AdvertisingDataType::PeripheralConnectionInterval)
    //            .expect("delete conn interval ad type")
    //    })
    //    .ok();

    hci_commands_queue
        .enqueue(|rc, _| {
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
                .expect("update service uuid list data")
        })
        .ok();

    //hci_commands_queue
    //    .enqueue(|rc, _| {
    //        let url_len = EDDYSTONE_URL.len();

    //        let mut service_data = [0u8; 24];
    //        service_data[0] = 6 + url_len as u8;
    //        service_data[1] = AdvertisingDataType::ServiceData as u8;

    //        // 16-bit Eddystone UUID
    //        service_data[2] = 0xAA;
    //        service_data[3] = 0xFE;

    //        service_data[4] = 0x10; // URL frame type
    //        service_data[5] = CALIBRATED_TX_POWER_AT_0_M;
    //        service_data[6] = EDDYSTONE_URL_PREFIX as u8;

    //        service_data[7..(7 + url_len)].copy_from_slice(EDDYSTONE_URL);

    //        rc.update_advertising_data(&service_data[..])
    //            .expect("update service data")
    //    })
    //    .ok();

    // hci_commands_queue
    //     .enqueue(|rc, _| {
    //         let mut service_uuid_list = [0u8;16*1 + 2];

    //         service_uuid_list[0] = 16*1 + 1;
    //         service_uuid_list[1] = AdvertisingDataType::Uuid128 as u8;

    //         for i in 0..16 {
    //             service_uuid_list[i+2] = UUID_ACCESSORY_INFORMATION[15-i];
    //         }

    //         rc.update_advertising_data(&service_uuid_list[..])
    //             .expect("update service uuid list data")
    //     })
    //     .ok();

    hci_commands_queue
        .enqueue(|rc, _| {
            let mut service_uuid_list = [0u8; 16 * 1 + 2];

            service_uuid_list[0] = 16 * 1 + 1;
            service_uuid_list[1] = AdvertisingDataType::Uuid128 as u8;

            for i in 0..16 {
                service_uuid_list[i + 2] = UUID_PAIRING_SERVICE[i];
            }

            rc.update_advertising_data(&service_uuid_list[..])
                .expect("update service uuid list data")
        })
        .ok();

    hci_commands_queue
        .enqueue(|rc, _| {
            let flags = [2, AdvertisingDataType::Flags as u8, 0x4 | 0x2];

            rc.update_advertising_data(&flags[..])
                .expect("update flags data")
        })
        .ok();
}
