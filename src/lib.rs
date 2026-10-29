#![no_std]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

use embedded_hal::digital;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital as digital_async;
use embedded_hal_async::i2c::I2c;

use crate::registers::Register;
pub use crate::registers::TouchGesture;

pub mod registers;

const CST816S_CHIP_ID: u8 = 0xb4;
const I2C_ADDRESS: u8 = 0x15;

/// CST816S driver errors
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<I2cError, InterruptPinError, ResetPinError> {
    /// I2C communication error
    I2c(I2cError),
    /// The chip ID read from the sensor was not 0xB4
    InvalidChipId(u8),
    /// Interrupt pin error
    InterruptPin(InterruptPinError),
    /// Reset Pin Error
    ResetPin(ResetPinError),
}

/// Touch event data
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchEvent {
    /// Gesture type
    pub gesture: TouchGesture,
    /// Finger
    pub finger: u8,
    /// Location
    pub location: Point,
}

/// Touch event data
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Point {
    /// X coordinate of touch
    pub x: i16,
    /// Y coordinate of touch
    pub y: i16,
}

/// Driver for the CST816S touch sensor
pub struct CST816S<I2C, InterruptPin, ResetPin> {
    address: u8,
    i2c: I2C,
    interrupt_pin: InterruptPin,
    reset_pin: ResetPin,
}

impl<I2C, InterruptPin, ResetPin> CST816S<I2C, InterruptPin, ResetPin>
where
    I2C: I2c,
    InterruptPin: digital_async::Wait,
    ResetPin: digital::OutputPin,
{
    /// Create a new CST816S driver instance with the default I2C address: 0x15
    pub fn new(i2c: I2C, interrupt_pin: InterruptPin, reset_pin: ResetPin) -> Self {
        CST816S {
            address: I2C_ADDRESS,
            i2c,
            interrupt_pin,
            reset_pin,
        }
    }

    /// Initialize the CST816S sensor by toggling the reset pin
    pub async fn init<T>(
        &mut self,
        mut delay: T,
    ) -> Result<(), Error<I2C::Error, InterruptPin::Error, ResetPin::Error>>
    where
        T: DelayNs,
    {
        self.reset_pin.set_low().map_err(Error::ResetPin)?;
        delay.delay_ms(20).await;
        self.reset_pin.set_high().map_err(Error::ResetPin)?;
        delay.delay_ms(50).await;
        self.check_chip_id().await?;
        Ok(())
    }

    /// Enable or Disable Double Taps
    /// Will send a specific gesture for double tap, but delays single tap response
    /// Default: Disabled
    pub async fn set_double_tap(
        &mut self,
        enabled: bool,
    ) -> Result<(), Error<I2C::Error, InterruptPin::Error, ResetPin::Error>> {
        self.set_motion_mask(enabled, 0).await
    }

    /// Enable or Disable Vertical Continuous mode
    /// Vertical swipes continually send interrupts
    /// Default: Disabled
    pub async fn set_vertical_continuous(
        &mut self,
        enabled: bool,
    ) -> Result<(), Error<I2C::Error, InterruptPin::Error, ResetPin::Error>> {
        self.set_motion_mask(enabled, 1).await
    }

    /// Enable or Disable Horizontal Continuous mode
    /// Horizontal swipes continuously send interrupts
    /// Default: Disabled
    pub async fn set_horizontal_continuous(
        &mut self,
        enabled: bool,
    ) -> Result<(), Error<I2C::Error, InterruptPin::Error, ResetPin::Error>> {
        self.set_motion_mask(enabled, 2).await
    }

    async fn set_motion_mask(
        &mut self,
        enabled: bool,
        bit: u8,
    ) -> Result<(), Error<I2C::Error, InterruptPin::Error, ResetPin::Error>> {
        self.wait_for_interrupt()
            .await
            .map_err(Error::InterruptPin)?;
        let mut buffer = [0u8; 1];
        self.write_read(Register::MotionMask, &mut buffer)
            .await
            .map_err(Error::I2c)?;
        if enabled {
            buffer[0] |= 1 << bit;
        } else {
            buffer[0] &= !(1 << bit);
        }
        self.set_register(Register::MotionMask, buffer[0])
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// Set Auto Sleep Time in seconds
    /// Default: 2 seconds
    pub async fn set_auto_sleep_time(
        &mut self,
        seconds: u8,
    ) -> Result<(), Error<I2C::Error, InterruptPin::Error, ResetPin::Error>> {
        self.set_register_at_interrupt(Register::AutoSleepTime, seconds)
            .await
    }

    /// Disable Auto Sleep
    /// Auto Sleep is enabled by default
    pub async fn disable_auto_sleep(
        &mut self,
    ) -> Result<(), Error<I2C::Error, InterruptPin::Error, ResetPin::Error>> {
        self.set_register_at_interrupt(Register::DisableAutoSleep, 1)
            .await
    }

    /// Enable Auto Sleep
    /// Auto Sleep is enabled by default
    pub async fn enable_auto_sleep(
        &mut self,
    ) -> Result<(), Error<I2C::Error, InterruptPin::Error, ResetPin::Error>> {
        self.set_register_at_interrupt(Register::DisableAutoSleep, 0)
            .await
    }

    async fn check_chip_id(
        &mut self,
    ) -> Result<(), Error<I2C::Error, InterruptPin::Error, ResetPin::Error>> {
        let mut buffer = [0u8; 1];
        self.write_read(Register::ChipId, &mut buffer)
            .await
            .map_err(Error::I2c)?;
        if buffer[0] != CST816S_CHIP_ID {
            return Err(Error::InvalidChipId(buffer[0]));
        }
        Ok(())
    }

    async fn wait_for_interrupt(&mut self) -> Result<(), InterruptPin::Error> {
        self.interrupt_pin.wait_for_falling_edge().await
    }

    /// Get the next touch event
    pub async fn wait_for_touch(
        &mut self,
    ) -> Result<TouchEvent, Error<I2C::Error, InterruptPin::Error, ResetPin::Error>> {
        self.wait_for_interrupt()
            .await
            .map_err(Error::InterruptPin)?;
        let mut buffer = [0u8; 6];
        self.write_read(Register::GestureId, &mut buffer)
            .await
            .map_err(Error::I2c)?;
        let gesture = TouchGesture::from(buffer[0]);
        let x = sign_extend(buffer[2], buffer[3]);
        let y = sign_extend(buffer[4], buffer[5]);
        let location = Point { x, y };
        Ok(TouchEvent {
            gesture,
            finger: buffer[1],
            location,
        })
    }

    async fn set_register_at_interrupt(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<I2C::Error, InterruptPin::Error, ResetPin::Error>> {
        self.wait_for_interrupt()
            .await
            .map_err(Error::InterruptPin)?;
        self.i2c
            .write(self.address, &[register as u8, value])
            .await
            .map_err(Error::I2c)
    }

    async fn set_register(&mut self, register: Register, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[register as u8, value]).await
    }

    async fn write_read(&mut self, register: Register, read: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c
            .write_read(self.address, &[register as u8], read)
            .await
    }
}

fn sign_extend(high_byte: u8, low_byte: u8) -> i16 {
    ((high_byte as i16) << 12) >> 4 | low_byte as i16
}
