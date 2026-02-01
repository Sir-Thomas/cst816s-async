#![no_std]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]


use embedded_graphics::prelude::Point;
use embedded_hal::digital;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital as digital_async;
use embedded_hal_async::i2c::I2c;

use crate::registers::Register;

mod registers;

const CST816S_CHIP_ID: u8 = 0xb4;
const I2C_ADDRESS: u8 = 0x15;

/// CST816S driver errors
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E, E2> {
    /// I2C communication error
    I2c(E),
    /// The chip ID read from the sensor was not 0x00
    InvalidChipId(u8),
    // Initialization error
    // InitializationError(u8),
    /// Interrupt pin error
    InterruptPin(E2),
}

/// Touch event data
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchEvent {
    /// Location
    pub location: Point,
    /// Gesture type
    pub gesture: TouchGesture,
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
    /// Double Tap - This doesn't seem to ever be reported by the CST816S
    DoubleTap = 0x0b,
    /// Long Press
    LongPress = 0x0c,
}

impl From<u8> for TouchGesture {
    fn from(value: u8) -> Self {
        match value {
            0x01 => TouchGesture::SwipeDown,
            0x02 => TouchGesture::SwipeUp,
            0x03 => TouchGesture::SwipeLeft,
            0x04 => TouchGesture::SwipeRight,
            0x05 => TouchGesture::Tap,
            0x0b => TouchGesture::DoubleTap,
            0x0c => TouchGesture::LongPress,
            _ => TouchGesture::None,
        }
    }
}


/// Driver for the CST816S touch sensor
pub struct CST816S<I2C, InterruptPin, ResetPin> {
    address: u8,
    i2c: I2C,
    interrupt_pin: InterruptPin,
    reset_pin: ResetPin,
}

impl<I2C, InterruptPin, ResetPin, E, E2> CST816S<I2C, InterruptPin, ResetPin>
where 
    I2C: I2c<Error = E>,
    InterruptPin: digital_async::Wait<Error = E2>,
    ResetPin: digital::OutputPin<Error = E2>,
{
    /// Create a new CST816S driver instance with the default I2C address: 0x18
    pub fn new(
        i2c: I2C,
        interrupt_pin: InterruptPin,
        reset_pin: ResetPin,
    ) -> Self {
        CST816S {
            address: I2C_ADDRESS,
            i2c,
            interrupt_pin,
            reset_pin,
        }
    }

    /// Initialize the CST816S sensor by toggling the reset pin
    pub async fn init<T>(&mut self, mut delay: T) -> Result<(), ResetPin::Error>
    where T: DelayNs
    {
        self.reset_pin.set_low()?;
        delay.delay_ms(20).await;
        self.reset_pin.set_high()?;
        delay.delay_ms(50).await;
        Ok(())
    }

    /// Check Chip ID
    /// The device will not respond until it has been touched.
    /// This function will wait for a touch event, then verify the chip ID before returning.
    pub async fn check_chip_id(&mut self) -> Result<(), Error<E, E2>> {
        self.wait_for_interrupt().await.map_err(Error::InterruptPin)?;
        let mut buffer = [0u8; 1];
        self.write_read(Register::ChipId, &mut buffer).await.map_err(Error::I2c)?;
        if buffer[0] != CST816S_CHIP_ID {
            return Err(Error::InvalidChipId(buffer[0]));
        }
        Ok(())
    }

    async fn wait_for_interrupt(&mut self) -> Result<(), InterruptPin::Error> {
        self.interrupt_pin.wait_for_high().await?;
        self.interrupt_pin.wait_for_low().await?;
        Ok(())
    }

    /// Get the next touch event
    pub async fn wait_for_touch(&mut self) -> Result<TouchEvent, Error<E, E2>> {
        self.wait_for_interrupt().await.map_err(Error::InterruptPin)?;
        let mut buffer = [0u8; 4];
        self.write_read(Register::TouchCoordinates, &mut buffer).await.map_err(Error::I2c)?;
        let coordinates = self.convert_coordinate_data(buffer);
        let mut buffer = [0u8; 1];
        self.write_read(Register::GestureId, &mut buffer).await.map_err(Error::I2c)?;
        let gesture = TouchGesture::from(buffer[0]);
        Ok(TouchEvent {
            location: coordinates,
            gesture,
        })
    }
    
    fn convert_coordinate_data(&self, buffer: [u8; 4]) -> Point {
        let x = self.format_coordinate(buffer[0], buffer[1]);
        let y = self.format_coordinate(buffer[2], buffer[3]);
        Point::new(x, y)
    }

    fn format_coordinate(&self, msb: u8, lsb: u8) -> i32 {
        let msb = ((msb & 0x0F) as i32) << 8;
        msb | lsb as i32
    }

    async fn write_read(&mut self, register: Register, read: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.write_read(self.address, &[register as u8], read).await?;
        Ok(())
    }
}