// This crate is `no_std`, only for
// tests not.
#![cfg_attr(not(test), no_std)]

use core::convert::{TryFrom, TryInto};

pub mod tlv;

#[derive(Debug)]
#[repr(u8)]
pub enum HapTlv {
    Value = 0x01,
    AdditionalAuthorizationData = 0x02,
    ParamOrigin = 0x03,
    /// Characteristic Type (UUID)
    ChrType = 0x04,
    /// Characteristic Type (UUID)
    ChrInstanceId = 0x05,
    /// Service Type (UUID)
    SvcType = 0x06,
    /// Service ID (2 Bytes)
    SvcId = 0x07,
    Ttl = 0x08,
    ReturnResponse = 0x09,
    /// HAP Characteristic Properties (2 Bytes)
    HapChrProperties = 0x0A,
    /// GATT User Description (UTF-8 String)
    GattUserDescription = 0x0B,
    /// GATT Format
    GattFormat = 0x0C,
    /// GATT Valid Range
    GattValidRange = 0x0D,
    /// HAP Step value
    HapStepValue = 0x0e,
    HapServiceProperties = 0x0f,
    HapLinkedServices = 0x10,
    HapValidValues = 0x11,
    HapValuesRange = 0x12,
}

impl From<HapTlv> for u8 {
    fn from(tlv: HapTlv) -> Self {
        tlv as u8
    }
}

#[derive(Debug)]
pub enum HapPdu<'a> {
    Request(HapRequest<'a>),
    Response(HapResponse<'a>),
}

impl HapPdu<'_> {
    pub fn parse(data: &[u8]) -> Result<HapPdu, Error> {
        // We need at least 1 byte for the control field

        require_length(data, 1)?;

        let control_field = data[0];

        let fragmented = if control_field & (1 << 7) == (1 << 7) {
            Fragmented::Continuation
        } else {
            Fragmented::First
        };

        assert!(
            fragmented == Fragmented::First,
            "Continuation not yet implemented"
        );

        let iid_size = if control_field & (1 << 4) == (1 << 4) {
            IidSize::Bit64
        } else {
            IidSize::Bit16
        };

        let request_type = if control_field & (1 << 1) == (1 << 1) {
            PduType::Response
        } else {
            PduType::Request
        };

        // check for reserved values in pdu type
        if 0b1100 & control_field != 0 {
            // Unsupported type of PDU.
            return Err(Error::UnsupportedPduType((control_field & 0b1110) >> 1));
        };

        match request_type {
            PduType::Request => Ok(HapPdu::Request(HapRequest::parse_after_control(
                &data[1..],
                iid_size,
            )?)),
            PduType::Response => {
                unimplemented!("Not yet implemented");
            }
        }
    }
}

#[derive(Debug)]
pub struct HapRequest<'a> {
    iid_size: IidSize,

    pub op_code: OpCode,

    pub tid: u8,

    pub char_id: u16,

    pub data: Option<&'a [u8]>,
}

impl HapRequest<'_> {
    fn parse_after_control(data: &[u8], iid_size: IidSize) -> Result<HapRequest, Error> {
        // The Request Header is at least 4 bytes (excluding the control field)

        require_length(data, 4)?;

        let op_code = OpCode::try_from(data[0])?;

        let tid = data[1];

        // Unwrap is safe, we know that we have at least 4 bytes
        let char_id: u16 = u16::from_le_bytes((&data[2..4]).try_into().unwrap());

        // TODO: Support data

        let pdu_data = if data.len() > 4 {
            // We need at least two bytes for the body length
            require_length(data, 6)?;

            let body_len: u16 = u16::from_le_bytes((&data[4..6]).try_into().unwrap());

            // Check for fragmentation
            if body_len as usize > data.len() - 4 {
                todo!(
                    "Fragmentation not implemented (body_len={}, data_len={})",
                    body_len,
                    data.len()
                );
            }

            Some(&data[6..])
        } else {
            None
        };

        Ok(HapRequest {
            iid_size,
            op_code,
            tid,
            char_id,
            data: pdu_data,
        })
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum Fragmented {
    First,
    Continuation,
}

enum PduType {
    Request,
    Response,
}

#[derive(Debug)]
enum IidSize {
    Bit16,
    Bit64,
}

/// HAP Status
///
/// See Table 7-37
#[derive(Debug, Copy, Clone)]
pub enum HapStatus {
    Success = 0x0,
    UnsupportedPdu = 0x1,
    MaxProcedures = 0x2,
    InsufficientAuthorization = 0x3,
    InvalidInstanceId = 0x4,
    InsufficientAuthentication = 0x5,
    InvalidRequest = 0x6,
}

#[derive(Debug)]
pub struct HapResponse<'a> {
    tid: u8,

    pub status: HapStatus,

    data: &'a [u8],
}

impl HapResponse<'_> {
    pub fn new(tid: u8, status: HapStatus, data: &[u8]) -> HapResponse<'_> {
        HapResponse { tid, status, data }
    }

    /// Write the response into a buffer.
    pub fn write_into(&self, buffer: &mut [u8]) -> Result<(), Error> {
        if self.size() > buffer.len() {
            return Err(Error::InsufficientBuffer);
        }

        // Data longer than u16 MAX is not supported by the
        // protocol
        if self.data.len() > (u16::MAX as usize) {
            panic!("Data for HapResponse has to be < u16::MAX");
        }

        // TODO: Support fragmentation,

        // Control field fixed to 2 for now (indicating unfragmented response)
        buffer[0] = 2;

        buffer[1] = self.tid;
        buffer[2] = self.status as u8;

        if self.data.len() > 0 {
            buffer[3] = self.data.len() as u8;
            buffer[4] = (self.data.len() >> 8) as u8;

            buffer[5..(5 + self.data.len())].copy_from_slice(&self.data);
        }

        Ok(())
    }

    /// Calculate the size of the response in bytes
    pub fn size(&self) -> usize {
        // Header consists of Control Field, TID, and Status
        //
        let header_len = 3;

        // The body is optional
        let body_len = if self.data.len() > 0 {
            self.data.len() + 2
        } else {
            0
        };

        header_len + body_len
    }
}

#[derive(Debug)]
pub enum Error {
    BadLength { actual: usize, min_required: usize },
    UnsupportedPduType(u8),
    UnknownOpCode(u8),
    InsufficientBuffer,
}

/// HAP Opcode, defined in Table 7-8
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum OpCode {
    CharacteristicSignatureRead,
    CharacteristicWrite,
    CharacteristicRead,
    CharacteristicTimedWrite,
    CharacteristicExecuteWrite,
    ServiceSignatureRead,
    CharacteristicConfiguration,
    ProtocolConfiguration,
}

impl TryFrom<u8> for OpCode {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use OpCode::*;

        let op_code = match value {
            1 => CharacteristicSignatureRead,
            2 => CharacteristicWrite,
            3 => CharacteristicRead,
            4 => CharacteristicTimedWrite,
            5 => CharacteristicExecuteWrite,
            6 => ServiceSignatureRead,
            7 => CharacteristicConfiguration,
            8 => ProtocolConfiguration,
            other => return Err(Error::UnknownOpCode(other)),
        };

        Ok(op_code)
    }
}

fn require_length(data: &[u8], minimum_required: usize) -> Result<(), Error> {
    if data.len() < minimum_required {
        Err(Error::BadLength {
            min_required: minimum_required,
            actual: data.len(),
        })
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_parsing_service_signature_pdu() {
        let rx_data = [0, 6, 1, 0x10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        let pdu = HapPdu::parse(&rx_data).unwrap();

        if let HapPdu::Request(request) = pdu {
            assert_eq!(request.op_code, OpCode::ServiceSignatureRead);
            assert_eq!(request.char_id, 0x10);
        } else {
            panic!("Expected HapPdu::Request, got {:?}", pdu);
        }
    }

    #[test]
    fn test_parsing_characteristic_signature_pdu() {
        let rx_data = [0, 1, 23, 35, 0];

        let pdu = HapPdu::parse(&rx_data).unwrap();

        if let HapPdu::Request(request) = pdu {
            assert_eq!(request.op_code, OpCode::CharacteristicSignatureRead);
            assert_eq!(request.char_id, 35);
        } else {
            panic!("Expected HapPdu::Request, got {:?}", pdu);
        }
    }

    #[test]
    fn test_parsing_pdu_too_small() {
        // A Request PDU needs at least 5 Bytes
        let rx_data = [0u8; 4];

        assert!(matches!(
            HapPdu::parse(&rx_data),
            Err(Error::BadLength { .. })
        ));
    }

    #[test]
    fn test_characteristic_write() {
        let rx_data = [
            0x0, 0x2, 0x5d, 0x22, 0x0, 0xb, 0x0, 0x1, 0x6, 0x0, 0x1, 0x0, 0x6, 0x1, 0x1, 0x9,
        ];

        let pdu = HapPdu::parse(&rx_data).unwrap();

        if let HapPdu::Request(request) = pdu {
            assert_eq!(request.tid, 0x5d);
        } else {
            panic!("Failed to parse HAP PDU");
        }
    }
}
