//! Support for TLV8 data structures
//!

pub enum Value<'a> {
    Bytes(&'a [u8]),
    Integer8(u8),
    Integer16(u16),
    Integer32(u32),
    String(&'a str),
}

impl<'a> From<&'a [u8]> for Value<'a> {
    fn from(data: &'a [u8]) -> Self {
        Value::Bytes(data)
    }
}

impl From<u8> for Value<'_> {
    fn from(data: u8) -> Self {
        Value::Integer8(data)
    }
}

impl From<u16> for Value<'_> {
    fn from(data: u16) -> Self {
        Value::Integer16(data)
    }
}
impl From<u32> for Value<'_> {
    fn from(data: u32) -> Self {
        Value::Integer32(data)
    }
}

pub struct Tlv<'a> {
    tlv_type: u8,
    value: Value<'a>,
}

impl<'a> Tlv<'a> {
    pub fn new(tlv_type: u8, value: impl Into<Value<'a>>) -> Self {
        Tlv {
            tlv_type,
            value: value.into(),
        }
    }

    pub fn write_into(&self, buffer: &mut [u8]) -> usize {
        match self.value {
            Value::Bytes(data) => self.write_raw_data(data, buffer),
            Value::Integer8(i) => {
                let data = i.to_le_bytes();
                self.write_raw_data(&data, buffer)
            }
            Value::Integer16(i) => {
                let data = i.to_le_bytes();
                self.write_raw_data(&data, buffer)
            }
            Value::Integer32(i) => {
                let data = i.to_le_bytes();
                self.write_raw_data(&data, buffer)
            }
            Value::String(s) => self.write_raw_data(s.as_bytes(), buffer),
        }
    }

    fn write_raw_data(&self, data: &[u8], buffer: &mut [u8]) -> usize {
        if data.len() < 0xff {
            buffer[0] = self.tlv_type;
            buffer[1] = data.len() as u8;
            buffer[2..(2 + data.len())].copy_from_slice(data);

            2 + data.len()
        } else {
            // PDU needs fragmentation
            let mut index = 0;

            for chunk in data.chunks(0xff) {
                buffer[index] = self.tlv_type;
                buffer[index + 1] = chunk.len() as u8;
                buffer[index + 2..index + 2 + chunk.len()].copy_from_slice(chunk);

                index += 2 + chunk.len();
            }

            index
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn write_bytes() {
        let tlv_value = Tlv {
            tlv_type: 12,
            value: Value::Bytes(&[0x12, 0x42]),
        };

        let mut buff = [0u8; 4];

        tlv_value.write_into(&mut buff);

        assert_eq!(buff, [12, 0x2, 0x12, 0x42]);
    }

    #[test]
    fn write_handle() {
        let tlv_value = Tlv {
            tlv_type: 12,
            value: Value::Integer16(0x123),
        };

        let mut buff = [0u8; 4];

        tlv_value.write_into(&mut buff);

        assert_eq!(buff, [12, 0x2, 0x23, 0x01]);
    }
}
