use std::convert::TryInto;

use anyhow::Context;
use bluer::{gatt::remote::CharacteristicWriteRequest, Adapter, Address};
use futures::StreamExt;
use homekit::HomekitCharacteristicUuid;
use scroll::Pread;

use homekit_ble::tlv::Tlv;

use crate::homekit::{CharProbs, HapRequest, HapResponse, HomekitServiceUuid};

mod homekit;

const APPLE_COID: u16 = 0x4c;

async fn show_device_info(adapter: &Adapter, addr: Address) -> Result<(), anyhow::Error> {
    let device = adapter
        .device(addr)
        .with_context(|| format!("Failed to open device with address {}", addr))?;

    println!(" - Address: {}", device.address());
    println!(" - Name:    {:?}", device.name().await?);

    if let Some(data) = device.manufacturer_data().await? {
        if let Some(homekit_data) = data.get(&APPLE_COID) {
            println!(" - Apple manufacturer data: {:02x?}", homekit_data);

            if is_homekit_device(homekit_data) {
                let mut homekit_device = HomekitDevice::new(device).await?;

                homekit_device.debug_print().await?;

                homekit_device.close().await?;
            }
        }
    } else {
        println!(" -- no manufacturer data");
    }

    println!(" ----");

    Ok(())
}

fn is_homekit_device(data: &[u8]) -> bool {
    let mut offset = 0;

    let type_: u8 = data[0];

    if type_ != 0x06 {
        println!(
            " - Unexpected Manufacturer Data, 'type' must be 0x06, but is {:#04x}",
            type_
        );
        return false;
    }

    offset += 1;

    let stl = data[offset];

    offset += 1;

    println!(" -- STL:  {:#04x}", stl);

    // Upper 3 bits must be set to 0b001:
    let subtype = stl >> 5;
    let remaining_len = stl & 0b11111;

    println!(" ---- subtype: {}", subtype);
    println!(" ---- len:     {}", remaining_len);

    println!(" -- SF:   {:#04x}", data[offset]);

    offset += 1;

    let device_id = &data[offset..offset + 6];
    println!(" -- Device ID: {:#02x?}", device_id);

    offset += 6;

    let acid: u16 = data
        .pread_with(offset, scroll::LE)
        .expect("Failed to parse ACID from manufacturer data");

    offset += 2;

    println!(" -- ACID: {:#06x}", acid);

    let gsn: u16 = data
        .pread_with(offset, scroll::LE)
        .expect("Failed to parse GSN.");

    offset += 2;

    println!(" -- GSN:  {:#06x}", gsn);

    let cn: u8 = data
        .pread_with(offset, scroll::LE)
        .expect("Failed to parse CN.");

    offset += 1;

    println!(" -- CN:   {:#04x}", cn);

    let cv: u8 = data
        .pread_with(offset, scroll::LE)
        .expect("Failed to parse CN.");

    offset += 1;

    println!(" -- CV:   {:#04x}", cv);

    true
}

struct HomekitDevice {
    device: bluer::Device,

    services: Vec<HomekitService>,

    tid: u8,
}

struct HomekitService {
    gatt_service: bluer::gatt::remote::Service,

    uuid: HomekitServiceUuid,

    instance_id: u16,

    characteristics: Vec<HomekitCharacteristic>,
}

struct HomekitCharacteristic {
    gatt_characteristic: bluer::gatt::remote::Characteristic,

    uuid: HomekitCharacteristicUuid,

    instance_id: u16,
}

impl HomekitCharacteristic {
    async fn new(
        characteristic: bluer::gatt::remote::Characteristic,
        uuid: bluer::Uuid,
    ) -> Result<Self, anyhow::Error> {
        let descriptors = characteristic.descriptors().await?;

        let mut instance_id = 0;

        for descriptor in descriptors {
            let descriptor_uuid = descriptor.uuid().await?;

            if descriptor_uuid == homekit::uuid::CHARACTERISTIC_INSTANCE_ID {
                let instance_id_raw = descriptor.read().await?;

                instance_id = u16::from_le_bytes(
                    instance_id_raw
                        .try_into()
                        .expect("Failed to read instance ID for characteristic"),
                )
            } else {
                println!("    \\-- {}", descriptor_uuid);
            }
        }

        Ok(Self {
            gatt_characteristic: characteristic,
            uuid: HomekitCharacteristicUuid::from_uuid(uuid),
            instance_id,
        })
    }
}

impl HomekitDevice {
    async fn new(device: bluer::Device) -> Result<Self, anyhow::Error> {
        if !device.is_connected().await? {
            device
                .connect()
                .await
                .context("Failed to connect to Homekit Device.")?;
        }

        let mut services = vec![];

        for service in device.services().await? {
            let uuid = HomekitServiceUuid::from_uuid(service.uuid().await?);

            let mut homekit_characteristics = vec![];

            let characteristics = service.characteristics().await?;

            let mut instance_id = None;

            for c in characteristics {
                let characteristic_uuid = c.uuid().await?;

                if characteristic_uuid == homekit::uuid::SERVICE_INSTANCE_ID {
                    let instance_id_raw = c.read().await?;

                    instance_id = Some(u16::from_le_bytes(
                        instance_id_raw
                            .try_into()
                            .expect("Did not read enough data for instance_id"),
                    ));
                } else {
                    let homekit_characteristic =
                        HomekitCharacteristic::new(c, characteristic_uuid).await?;

                    homekit_characteristics.push(homekit_characteristic);
                }
            }

            if let Some(instance_id) = instance_id {
                services.push(HomekitService {
                    gatt_service: service,
                    uuid,
                    instance_id,
                    characteristics: homekit_characteristics,
                });
            } else {
                println!(" ! No homekit service (missing instance ID): {:?} !", uuid)
            }
        }

        Ok(Self {
            device,
            services,
            tid: 1,
        })
    }

    async fn debug_print(&mut self) -> Result<(), anyhow::Error> {
        for service in &self.services {
            println!(
                "  \\-- Service {:?}  - Instance {}",
                service.uuid, service.instance_id
            );

            for characteristic in &service.characteristics {
                println!(
                    "   |-- {:?} - Instance {}",
                    characteristic.uuid, characteristic.instance_id
                );

                if HomekitCharacteristicUuid::ServiceSignature != characteristic.uuid {
                    // Read the signature of the characteristic

                    let hap_request = HapRequest {
                        iid_size: homekit::IidSize::Bit16,
                        opcode: homekit::Opcode::CharacteristicSignatureRead,
                        tid: self.tid,
                        char_id: characteristic.instance_id,
                        data: vec![],
                    };

                    self.tid += 1;

                    let hap_request_data = hap_request.to_bytes();

                    let write_request = CharacteristicWriteRequest {
                        offset: 0,
                        op_type: bluer::gatt::WriteOp::Request,
                        prepare_authorize: true,
                        ..Default::default()
                    };

                    log::trace!("    -- Characteristic Request: {:x?}", hap_request_data);

                    characteristic
                        .gatt_characteristic
                        .write_ext(&hap_request_data, &write_request)
                        .await
                        .context("Failed to write data to characteristic")?;

                    let response = characteristic
                        .gatt_characteristic
                        .read()
                        .await
                        .context("Failed to read data from characteristic")?;

                    log::trace!("    -- Characteristic Response: {:x?}", response);

                    match HapResponse::parse(&response) {
                        Ok(parsed_response) => {
                            log::trace!("    -- Parsed: {:x?}", parsed_response);
                            println!("   \\-  Signature Read: {:?}", parsed_response.status);

                            if let Some(data) = parsed_response.data {
                                let tlvs = Tlv::parse(&data);

                                for tlv in tlvs {
                                    log::trace!("TLV: {:?}", tlv);
                                    match tlv.tlv_type {
                                        0x04 => {
                                            // UUID (Char Type)
                                            let uuid = tlv_to_uuid(&tlv)?;
                                            println!("    |-- UUID:      {:x?}", uuid);
                                        }
                                        0x07 => {
                                            // SVC ID
                                            let id = tlv_to_u16(&tlv)?;
                                            println!("    |-- SVC ID:    {}", id);
                                        }
                                        0x06 => {
                                            // SVC Type
                                            let uuid = tlv_to_uuid(&tlv)?;
                                            println!("    |-- SVC Type:  {:x?}", uuid);
                                        }
                                        0x0A => {
                                            // Characteristic properties
                                            let prop_value = tlv_to_u16(&tlv)?;
                                            let probs = CharProbs::from_bits(prop_value).ok_or_else(|| anyhow::anyhow!("Failed to parse HAP Characteristic properties from {}", prop_value))?;
                                            println!("    |-- Char Prop: {:?}", probs);
                                        }
                                        0x0B => {
                                            // UTF-8 User description
                                        }
                                        0x0C => {
                                            // GATT Format
                                            println!("    |-- Gatt Fmt:  {:x?}", tlv.value);
                                        }
                                        0x0D => {
                                            // GATT Valid Range
                                        }
                                        0x0E => {
                                            // HAP Step Value
                                        }
                                        n => println!("Unknown TLV type {}", n),
                                    }
                                }
                            }
                        }

                        Err(e) => println!("    -- Failed to parse: {:?}", e),
                    };
                }
            }
        }

        Ok(())
    }

    async fn close(self) -> Result<(), anyhow::Error> {
        self.device.disconnect().await?;

        Ok(())
    }
}

fn tlv_to_uuid(tlv: &Tlv) -> anyhow::Result<bluer::Uuid> {
    match &tlv.value {
        homekit_ble::tlv::Value::Bytes(bytes) => {
            let raw_bytes: [u8; 16] = (*bytes).try_into()?;
            let raw_value = u128::from_le_bytes(raw_bytes);
            let uuid = bluer::Uuid::from_u128(raw_value);

            Ok(uuid)
        }
        other => anyhow::bail!("Unexpected TLV value for UUID {:?}", other),
    }
}

fn tlv_to_u16(tlv: &Tlv) -> anyhow::Result<u16> {
    match &tlv.value {
        homekit_ble::tlv::Value::Bytes(bytes) => {
            let raw_bytes: [u8; 2] = (*bytes).try_into()?;
            let raw_value = u16::from_le_bytes(raw_bytes);

            Ok(raw_value)
        }
        other => anyhow::bail!("Unexpected TLV value for u16 {:?}", other),
    }
}

async fn characteristic_signature_read(characteristic: &HomekitCharacteristic) {}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    pretty_env_logger::init();

    let session = bluer::Session::new().await?;

    let adapter_names = session.adapter_names().await?;

    let adapter = session.adapter(adapter_names.first().expect("No Adapter found."))?;

    println!("Opened adapter: {:?}", adapter);

    let mut discover_events = adapter.discover_devices().await?;

    while let Some(evt) = discover_events.next().await {
        match evt {
            bluer::AdapterEvent::DeviceAdded(addr) => {
                if let Err(err) = show_device_info(&adapter, addr).await {
                    eprintln!("Error accessing device information: {:?}", err);
                }
            }
            bluer::AdapterEvent::DeviceRemoved(addr) => {
                println!("Device removed: {}", addr);
            }
            // Ignore properties for now
            bluer::AdapterEvent::PropertyChanged(_) => (),
        }
    }

    Ok(())
}
