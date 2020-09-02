//! UUIDs used in Homekit

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
pub const UUID_ACCESSORY_INFORMATION: [u8; 16] = apple_uuid(0x3E);

// public.hap.characteristic.firmware.revision (9.40, p. 177)
pub const UUID_ACCESSORY_INFORMATION_FIRMWARE_REVISION: [u8; 16] = apple_uuid(0x52);

// public.hap.characteristic.hardware.revision (9.41, p. 178)
pub const UUID_ACCESSORY_INFORMATION_HARDWARE_REVISION: [u8; 16] = apple_uuid(0x53);

// public.hap.characteristic.identify (9.45, p. 180)
pub const UUID_ACCESSORY_INFORMATION_IDENTIFY: [u8; 16] = apple_uuid(0x14);

/// public.hap.characteristic.manufacturer (9.58, p. 187)
pub const UUID_ACCESSORY_INFORMATION_MANUFACTURER: [u8; 16] = apple_uuid(0x20);

/// public.hap.characteristic.model (9.59, p. 187)
pub const UUID_ACCESSORY_INFORMATION_MODEL: [u8; 16] = apple_uuid(0x21);

/// public.hap.characteristic.name (9.62, p. 188)
pub const UUID_CHARACTERISTIC_NAME: [u8; 16] = apple_uuid(0x23);

// public.hap.characteristic.firmware.revision (9.87, p. 201)
pub const UUID_ACCESSORY_INFORMATION_SERIAL_NUMBER: [u8; 16] = apple_uuid(0x30);

// UUID for HAP Protocol Information service
// public.hap.service.protocol.information.service
pub const UUID_PROTOCOL_INFORMATION: [u8; 16] = [
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xA2, 0x00, 0x00, 0x00,
];

pub const UUID_PAIRING_SERVICE: [u8; 16] = apple_uuid(0x55);

pub const UUID_PAIRING_SETUP: [u8; 16] = apple_uuid(0x4C);
pub const UUID_PAIRING_VERIFY: [u8; 16] = apple_uuid(0x4E);

pub const UUID_PAIRING_FEATURES: [u8; 16] = apple_uuid(0x4F);
pub const UUID_PAIRING_PAIRINGS: [u8; 16] = apple_uuid(0x50);

pub const UUID_LIGHTBULB_SERVICE: [u8; 16] = apple_uuid(0x43);

// characteristic.on
pub const UUID_CHARACTERISTIC_ON: [u8; 16] = apple_uuid(0x25);

/// UUID for the Service Instance ID.
///
/// Every HAP service must have a readonly service instance ID iwth this UUID.
pub const UUID_SERVICE_INSTANCE: [u8; 16] = [
    0xD1, 0xA0, 0x83, 0x50, 0x00, 0xAA, 0xD3, 0x87, 0x17, 0x48, 0x59, 0xA7, 0x5D, 0xE9, 0x04, 0xE6,
];

// UUID
pub const UUID_VERSION_CHARACTERISTIC: [u8; 16] = [
    // "00000037-0000-1000-8000-0026BB765291"
    0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00,
];

pub const UUID_CHARACTERISTIC_ID: [u8; 16] = [
    // DC46F0FE-81D2-4616-B5D9-6A BD D7 96 93 9A
    0x9A, 0x93, 0x96, 0xD7, 0xBD, 0x6A, 0xD9, 0xB5, 0x16, 0x46, 0xD2, 0x81, 0xFE, 0xF0, 0x46, 0xDC,
];

/// Service Signature HAP characteristic
pub const UUID_SERVICE_SIGNATURE: [u8; 16] = apple_uuid(0xA5);
