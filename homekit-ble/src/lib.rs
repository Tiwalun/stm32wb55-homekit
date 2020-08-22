// This crate is `no_std`, only for
// tests not.
#![cfg_attr(not(test), no_std)]

use core::convert::{TryFrom, TryInto};

#[derive(Debug)]
pub enum HapPdu<'a> {
    Request(HapRequest<'a>),
    Response(HapResponse),
}

impl HapPdu<'_> {
    pub fn parse(data: &[u8]) -> Result<HapPdu, Error> {
        // We need at least 1 byte for the control field

        let control_field = data.get(0).ok_or(Error::BadLength)?;

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

    op_code: OpCode,

    tid: u8,

    char_id: u16,

    data: Option<&'a [u8]>,
}

impl HapRequest<'_> {
    fn parse_after_control(data: &[u8], iid_size: IidSize) -> Result<HapRequest, Error> {
        // The Request Header is at least 4 bytes (excluding the control field)

        if data.len() < 4 {
            return Err(Error::BadLength);
        }

        let op_code = OpCode::try_from(data[0])?;

        let tid = data[1];

        // Unwrap is safe, we know that we have at least 4 bytes
        let char_id: u16 = u16::from_le_bytes((&data[2..4]).try_into().unwrap());

        // TODO: Support data

        Ok(HapRequest {
            iid_size,
            op_code,
            tid,
            char_id,
            data: None,
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

#[derive(Debug)]
pub struct HapResponse {}

#[derive(Debug)]
pub enum Error {
    BadLength,
    UnsupportedPduType(u8),
    UnknownOpCode(u8),
}

/// HAP Opcode, defined in Table 7-8
#[derive(Debug)]
enum OpCode {
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

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_parsing_pdu() {
        let rx_data = [0, 6, 1, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        let _pdu = HapPdu::parse(&rx_data).unwrap();
    }

    #[test]
    fn test_parsing_pdu_too_small() {
        // A Request PDU needs at least 5 Bytes
        let rx_data = [0u8; 4];

        assert!(matches!(HapPdu::parse(&rx_data), Err(Error::BadLength)));
    }
}
