use crate::{
    ble::{perform_command, Characteristic, Service},
    uuid::{UUID_CHARACTERISTIC_ID, UUID_SERVICE_INSTANCE},
};
/// HAP Characteristic
use bitflags::bitflags;
use homekit_ble::HapRequest;
use rtt_target::rprintln;
use stm32wb55::{
    event::AttributeHandle,
    gatt::{
        CharacteristicEvent, CharacteristicProperty, Commands as GattCommands, DescriptorHandle,
        DescriptorValueParameters, ServiceType, Uuid,
    },
};

pub struct HapService {
    /// Bluetooth handle of the service
    service: Service,

    /// UUID of the Homekit Service
    pub uuid: [u8; 16],

    pub instance_id: u16,

    instance_id_characteristic: Characteristic,
}

impl HapService {
    pub fn new(
        uuid: [u8; 16],
        max_attribute_records: u8,
        instance_id: u16,
    ) -> Result<HapService, ()> {
        let service = Service::new(
            ServiceType::Primary,
            Uuid::Uuid128(uuid),
            max_attribute_records,
        )?;

        let instance_id_characteristic = service.add_characteristic(
            &Uuid::Uuid128(UUID_SERVICE_INSTANCE),
            CharacteristicProperty::READ,
            CharacteristicEvent::CONFIRM_READ | CharacteristicEvent::ATTRIBUTE_WRITE,
            2,
            false,
        )?;

        instance_id_characteristic.set_value(&instance_id.to_le_bytes())?;

        Ok(HapService {
            service,
            uuid,
            instance_id,
            instance_id_characteristic,
        })
    }

    pub(crate) fn contains_handle(&self, handle: AttributeHandle) -> bool {
        self.service.contains_handle(handle)
    }
}

type CharWriteHandler = fn(&mut HapCharacteristic, tid: u8, data: Option<&[u8]>) -> Result<(), ()>;

pub struct HapCharacteristic {
    pub debug_name: &'static str,

    characteristic: Characteristic,
    characteristic_id: DescriptorHandle,

    pub uuid: [u8; 16],

    pub instance_id: u16,

    /// Characteristic properties,
    /// see section 7.4.4.6.1
    pub properties: HapProperties,

    pub format: GattFormat,

    pub unit: Unit,

    read_handler: fn(&mut Self, request: &HapRequest) -> Result<(), ()>,

    write_handler: CharWriteHandler,
}

impl HapCharacteristic {
    pub fn handle_read(&mut self, request: &HapRequest) -> Result<(), ()> {
        (self.read_handler)(self, request)
    }

    pub fn handle_write(&mut self, tid: u8, data: Option<&[u8]>) -> Result<(), ()> {
        rprintln!("Write to {}", self.debug_name);
        (self.write_handler)(self, tid, data)
    }
}

bitflags! {
    pub struct HapProperties: u16 {
        const READ = 0x1;
        const WRITE = 0x2;
        const ADDITIONAL_AUTHORIZATION = 0x4;
        const TIMED_WRITE = 0x8;
        const SECURE_READ = 0x10;
        const SECURE_WRITE = 0x20;
        const HIDDEN = 0x40;
        const NOTIFY_CONNECTED = 0x80;
        const NOTIFY_DISCONNECTED = 0x100;
        const NOTIFY_BROADCAST = 0x200;
    }

}

#[derive(Debug, Copy, Clone)]
#[repr(u16)]
#[allow(dead_code)]
pub enum Unit {
    Celsius = 0x272f,
    ArcDegress = 0x2763,
    Percentage = 0x27ad,
    Unitless = 0x2700,
    Lux = 0x2731,
    Seconds = 0x2703,
}

impl Default for Unit {
    fn default() -> Self {
        Unit::Unitless
    }
}

#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
pub enum GattFormat {
    Bool = 0x01,
    Uint8 = 0x04,
    Uint16 = 0x06,
    Uint32 = 0x08,
    Uint64 = 0x0A,
    Int = 0x10,
    Float = 0x14,
    String = 0x19,
    Data = 0x1B,
}

impl HapCharacteristic {
    pub fn build(
        name: &'static str,
        service: &HapService,
        instance_id: u16,
        uuid: [u8; 16],
        ble_properties: CharacteristicProperty,
        hap_properties: HapProperties,
        format: GattFormat,
        characteristic_len: usize,
        read_handler: fn(&mut Self, request: &HapRequest) -> Result<(), ()>,
        write_handler: CharWriteHandler,
    ) -> Result<Self, ()> {
        // let (is_variable, characteristic_len) = match characteristic_len {
        //     CharacteristicLength::Fixed(len) => (false, len),
        //     CharacteristicLength::Variable(len) => (true, len),
        // };

        let characteristic = service.service.add_characteristic(
            &Uuid::Uuid128(uuid),
            ble_properties,
            CharacteristicEvent::CONFIRM_READ | CharacteristicEvent::ATTRIBUTE_WRITE,
            characteristic_len,
            true,
        )?;

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
            debug_name: name,
            characteristic,
            uuid,
            instance_id,
            properties: hap_properties,
            characteristic_id: descriptor_handle,
            format,
            unit: Unit::default(),
            read_handler,
            write_handler,
        })
    }

    pub fn set_value(&self, value: &[u8]) -> Result<(), ()> {
        /*
        rprintln!(
            "{:?}: value={:x?}",
            self.characteristic.characteristic,
            value
        );
        */

        self.characteristic.set_value(value)
    }
}
