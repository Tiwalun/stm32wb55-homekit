use anyhow::{bail, ensure};
use bitflags::bitflags;
use bluer::Uuid;
use std::convert::TryInto;

bitflags! {
    pub struct CharProbs: u16 {
        const READ = 0x0001;
        const WRITE = 0x0002;
        const ADDITIONAL_AUTHORIZATION = 0x0004;
        const TIMED_WRITE = 0x0008;
        const SECURE_READ = 0x0010;
        const SECURE_WRITE = 0x0020;
        const HIDDEN = 0x40;
        const EVENTS_CONNECTED = 0x80;
        const EVENTS_DISCONNECTED = 0x100;
        const BROADCAST = 0x200;
    }
}

#[derive(Debug, PartialEq)]
pub enum HomekitServiceUuid {
    AccessoryInformation,
    Lightbulb,
    Pairing,
    ProtocolInformation,
    TemperatureSensor,
    Battery,
    Unknown(Uuid),
}

impl HomekitServiceUuid {
    pub fn from_uuid(uuid: Uuid) -> Self {
        match uuid {
            uuid::PAIRING_SERVICE => HomekitServiceUuid::Pairing,
            uuid::ACCESSORY_INFORMATION => HomekitServiceUuid::AccessoryInformation,
            uuid::LIGHTBULB_SERVICE => HomekitServiceUuid::Lightbulb,
            uuid::PROTOCOL_INFORMATION => HomekitServiceUuid::ProtocolInformation,
            uuid::TEMPERATURE_SENSOR => HomekitServiceUuid::TemperatureSensor,
            uuid::BATTERY_SENSOR => HomekitServiceUuid::Battery,
            uuid => HomekitServiceUuid::Unknown(uuid),
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum HomekitCharacteristicUuid {
    Name,
    PairingPairSetup,
    PairingPairVerify,
    PairingFeatures,
    PairingPairings,
    ProtocolVersion,
    FirmwareRevision,
    HardwareRevision,
    ServiceSignature,
    Identify,
    Model,
    Manufacturer,
    SerialNumber,
    On,
    Unknown(Uuid),
}

impl HomekitCharacteristicUuid {
    pub fn from_uuid(uuid: Uuid) -> Self {
        match uuid {
            uuid::PAIRING_SETUP => HomekitCharacteristicUuid::PairingPairSetup,
            uuid::PAIRING_VERIFY => HomekitCharacteristicUuid::PairingPairVerify,
            uuid::PAIRING_FEATURES => HomekitCharacteristicUuid::PairingFeatures,
            uuid::PAIRING_PAIRINGS => HomekitCharacteristicUuid::PairingPairings,
            uuid::PROTOCOL_VERSION => HomekitCharacteristicUuid::ProtocolVersion,
            uuid::FIRMWARE_REVISION => HomekitCharacteristicUuid::FirmwareRevision,
            uuid::NAME => HomekitCharacteristicUuid::Name,
            uuid::IDENTIFY => HomekitCharacteristicUuid::Identify,
            uuid::MANUFACTURER => HomekitCharacteristicUuid::Manufacturer,
            uuid::SERIAL_NUMBER => HomekitCharacteristicUuid::SerialNumber,
            uuid::HARDWARE_REVISION => HomekitCharacteristicUuid::HardwareRevision,
            uuid::MODEL => HomekitCharacteristicUuid::Model,
            uuid::SERVICE_SIGNATURE => HomekitCharacteristicUuid::ServiceSignature,
            uuid::ON => HomekitCharacteristicUuid::On,
            uuid => HomekitCharacteristicUuid::Unknown(uuid),
        }
    }
}

#[derive(Debug)]
pub enum IidSize {
    Bit16,
    Bit64,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum Opcode {
    CharacteristicSignatureRead = 0x1,
    CharacteristicWrite = 0x2,
}

#[repr(u8)]
#[derive(Debug)]
pub enum HapStatus {
    Success = 0x0,
    UnsupportedPdu = 0x1,
    MaxProcedures = 0x2,
    InsufficientAuthorization = 0x3,
    InvalidInstanceId = 0x4,
    InsufficientAuthentication = 0x5,
    InvalidRequest = 0x06,
}

impl HapStatus {
    fn parse(status: u8) -> anyhow::Result<Self> {
        let status = match status {
            0 => HapStatus::Success,
            1 => HapStatus::UnsupportedPdu,
            2 => HapStatus::MaxProcedures,
            3 => HapStatus::InsufficientAuthorization,
            4 => HapStatus::InvalidInstanceId,
            5 => HapStatus::InsufficientAuthentication,
            6 => HapStatus::InvalidRequest,
            other => bail!("Invalid Status for HAP response: {}", other),
        };

        Ok(status)
    }
}

pub struct HapRequest {
    pub iid_size: IidSize,

    pub opcode: Opcode,

    pub tid: u8,

    pub char_id: u16,
    pub data: Vec<u8>,
}

impl HapRequest {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut control_field = 0u8;

        match self.iid_size {
            IidSize::Bit16 => (),
            IidSize::Bit64 => control_field |= 1 << 4,
        };

        // TODO: support fragmentation
        assert!(self.data.len() <= 0xffff);

        // This is a request
        control_field |= 0 << 1;

        let id_bytes = self.char_id.to_le_bytes();

        let data_len = self.data.len() as u16;

        let data_len_bytes = data_len.to_le_bytes();

        let mut request = vec![
            control_field,
            self.opcode as u8,
            self.tid,
            id_bytes[0],
            id_bytes[1],
            data_len_bytes[0],
            data_len_bytes[1],
        ];

        request.extend_from_slice(&self.data);

        request
    }
}

#[derive(Debug)]
pub struct HapResponse {
    control_field: u8,
    pub tid: u8,
    pub status: HapStatus,

    pub data: Option<Vec<u8>>,
}

impl HapResponse {
    pub fn parse(data: &[u8]) -> anyhow::Result<Self> {
        let control = data[0];
        let tid = data[1];

        ensure!(
            control & (1 << 1) == (1 << 1),
            "Response bit in control header not set."
        );

        let status = HapStatus::parse(data[2])?;

        match data.len() {
            3 => Ok(Self {
                control_field: control,
                tid,
                status,
                data: None,
            }),
            4 => {
                bail!("Mistake in data length: Got 4 but should be 3 or larger than 5")
            }
            len => {
                let encoded_len = u16::from_le_bytes(
                    data[3..5]
                        .try_into()
                        .expect("Failed to parse encoded length"),
                ) as usize;

                ensure!(len - 5 == encoded_len, "Mistake in encoded length.");

                Ok(Self {
                    control_field: control,
                    tid,
                    status,
                    data: Some((&data[5..]).to_owned()),
                })
            }
        }
    }
}

pub mod uuid {
    use bluer::Uuid;

    pub const SERVICE_INSTANCE_ID: Uuid = Uuid::from_bytes([
        0xE6, 0x04, 0xE9, 0x5D, 0xA7, 0x59, 0x48, 0x17, 0x87, 0xD3, 0xAA, 0x00, 0x50, 0x83, 0xA0,
        0xD1,
    ]);

    pub const CHARACTERISTIC_INSTANCE_ID: Uuid = Uuid::from_bytes([
        0xDC, 0x46, 0xF0, 0xFE, 0x81, 0xD2, 0x46, 0x16, 0xB5, 0xD9, 0x6A, 0xBD, 0xD7, 0x96, 0x93,
        0x9A,
    ]);

    pub const ACCESSORY_INFORMATION: Uuid = apple_uuid(0x3E);

    // public.hap.service.sensor.temperature
    pub const TEMPERATURE_SENSOR: Uuid = apple_uuid(0x8A);

    // public.hap.service.sensor.battery
    pub const BATTERY_SENSOR: Uuid = apple_uuid(0x96);

    // public.hap.service.pairing
    pub const PAIRING_SERVICE: Uuid = apple_uuid(0x55);

    /// public.hap.characteristic.pairing.pair-setup
    pub const PAIRING_SETUP: Uuid = apple_uuid(0x4C);

    /// public.hap.characteristic.pairing.pair-verify
    pub const PAIRING_VERIFY: Uuid = apple_uuid(0x4E);

    /// public.hap.characteristic.pairing.pair-features
    pub const PAIRING_FEATURES: Uuid = apple_uuid(0x4F);

    /// public.hap.characteristic.pairing.pair-pairings
    pub const PAIRING_PAIRINGS: Uuid = apple_uuid(0x50);

    pub const LIGHTBULB_SERVICE: Uuid = apple_uuid(0x43);

    pub const PROTOCOL_INFORMATION: Uuid = apple_uuid(0xA2);

    /// public.hap.characteristic.version
    pub const PROTOCOL_VERSION: Uuid = apple_uuid(0x37);

    /// public.hap.characteristic.firmware.revision
    pub const FIRMWARE_REVISION: Uuid = apple_uuid(0x52);

    /// public.hap.characteristic.name
    pub const NAME: Uuid = apple_uuid(0x23);

    /// public.hap.characteristic.identify
    pub const IDENTIFY: Uuid = apple_uuid(0x14);

    /// public.hap.characteristic.manufacturer
    pub const MANUFACTURER: Uuid = apple_uuid(0x20);

    /// public.hap.characteristic.model
    pub const MODEL: Uuid = apple_uuid(0x21);

    /// public.hap.characteristic.on
    pub const ON: Uuid = apple_uuid(0x25);

    /// public.hap.characteristic.serial_number
    pub const SERIAL_NUMBER: Uuid = apple_uuid(0x30);

    /// public.hap.characteristic.hardware.revision
    pub const HARDWARE_REVISION: Uuid = apple_uuid(0x53);

    pub const SERVICE_SIGNATURE: Uuid = apple_uuid(0xA5);

    const fn apple_uuid(value: u32) -> Uuid {
        let value_bytes = value.to_be_bytes();

        let uuid_bytes = [
            value_bytes[0],
            value_bytes[1],
            value_bytes[2],
            value_bytes[3],
            0x00,
            0x00,
            0x10,
            0x00,
            0x80,
            0x00,
            0x00,
            0x26,
            0xbb,
            0x76,
            0x52,
            0x91,
        ];

        let default_uuid = Uuid::from_bytes(uuid_bytes);

        default_uuid
    }
}
