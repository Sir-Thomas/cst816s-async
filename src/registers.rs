//! Register definitions for the CST816S sensor
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum Register {
    GestureId = 0x01,
    TouchCoordinates = 0x03,
    ChipId = 0xa7,
}

impl From<Register> for u8 {
    fn from(reg: Register) -> Self {
        reg as u8
    }
}