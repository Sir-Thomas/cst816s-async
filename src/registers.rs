#![warn(missing_docs)]
//! Register definitions for the CST816S sensor

#[allow(dead_code)]
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum Register {
    GestureId = 0x01,
    Finger = 0x02,
    TouchCoordinates = 0x03,
    ChipId = 0xA7,
    MotionMask = 0xEC,
    AutoSleepTime = 0xF9,
    DisableAutoSleep = 0xFE,
}

impl From<Register> for u8 {
    fn from(reg: Register) -> Self {
        reg as u8
    }
}

/// Touch gesture types
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TouchGesture {
    /// No gesture detected
    None = 0x00,
    /// Swipe Down
    SwipeDown = 0x01,
    /// Swipe up
    SwipeUp = 0x02,
    /// Swipe left
    SwipeLeft = 0x03,
    /// Swipe right
    SwipeRight = 0x04,
    /// Tap
    Tap = 0x05,
    /// Double Tap
    DoubleTap = 0x0B,
    /// Long Press
    LongPress = 0x0C,
    /// Unknown Gesture
    Unknown(u8),
}

impl From<u8> for TouchGesture {
    fn from(value: u8) -> Self {
        match value {
            0x00 => Self::None,
            0x01 => Self::SwipeDown,
            0x02 => Self::SwipeUp,
            0x03 => Self::SwipeLeft,
            0x04 => Self::SwipeRight,
            0x05 => Self::Tap,
            0x0B => Self::DoubleTap,
            0x0C => Self::LongPress,
            gesture_id => Self::Unknown(gesture_id),
        }
    }
}
