use core::convert::TryInto;
use embedded_hal::digital::v2 as hal;
use esp_idf_sys::{
    gpio_input_get, gpio_input_get_high, gpio_mode_t_GPIO_MODE_INPUT, gpio_mode_t_GPIO_MODE_OUTPUT,
    gpio_pad_pulldown, gpio_pad_pullup, gpio_pad_select_gpio, gpio_set_direction, gpio_set_level,
    GPIO,
};

#[derive(Clone, Debug)]
pub enum Error {}

pub enum PullDir {
    Up,
    Down,
}

pub struct InputPin {
    which: u8,
}

impl InputPin {
    pub unsafe fn new(which: u8, pulldir: PullDir) -> Self {
        gpio_pad_select_gpio(which.try_into().unwrap());

        match pulldir {
            PullDir::Up => gpio_pad_pullup(which),
            PullDir::Down => gpio_pad_pulldown(which),
        };

        gpio_set_direction(which.into(), gpio_mode_t_GPIO_MODE_INPUT);

        Self { which }
    }

    pub fn which(&self) -> u8 {
        self.which
    }
}

impl hal::InputPin for InputPin {
    type Error = Error;

    fn is_high(&self) -> Result<bool, Self::Error> {
        let (bitmask, bit) = match self.which {
            0..=31 => (unsafe { gpio_input_get() }, self.which),
            32..=39 => (unsafe { gpio_input_get_high() }, self.which - 32),
            _ => unreachable!(),
        };

        Ok((bitmask & (1 << bit)) != 0)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        self.is_high().map(|x| !x)
    }
}

pub struct OutputPin {
    which: u8,
}

impl OutputPin {
    pub unsafe fn new(which: u8) -> Self {
        Self::with_initial(which, false)
    }

    pub unsafe fn with_initial(which: u8, initial_high: bool) -> Self {
        gpio_pad_select_gpio(which);
        gpio_set_level(which.into(), initial_high as u32);
        gpio_set_direction(which.into(), gpio_mode_t_GPIO_MODE_OUTPUT);
        Self { which }
    }

    pub fn which(&self) -> u8 {
        self.which
    }
}

impl hal::OutputPin for OutputPin {
    type Error = Error;

    fn set_high(&mut self) -> Result<(), Error> {
        unsafe {
            gpio_set_level(self.which.into(), 1);
        }
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Error> {
        unsafe {
            gpio_set_level(self.which.into(), 0);
        }
        Ok(())
    }
}

/// Set output level high for multiple GPIO pins, 0 to 31.
/// Each GPIO pin corresponds to a bit in the bitmask parameter.
#[inline]
pub unsafe fn set_multiple_high0(bitmask: u32) {
    GPIO.out_w1ts = bitmask;
}

/// Set output level high for multiple GPIO pins, 32 and above.
/// Each GPIO pin corresponds to a bit in the bitmask parameter, starting at 32.
#[inline]
pub unsafe fn set_multiple_high1(mut bitmask: u32) {
    // Pins 34+ are input-only
    bitmask &= 0x3;

    GPIO.out1_w1ts.val = bitmask;
}

/// Set output level low for multiple GPIO pins, 0 to 31.
/// Each GPIO pin corresponds to a bit in the bitmask parameter.
#[inline]
pub unsafe fn set_multiple_low0(bitmask: u32) {
    GPIO.out_w1tc = bitmask;
}

/// Set output level low for multiple GPIO pins, 32 and above.
/// Each GPIO pin corresponds to a bit in the bitmask parameter, starting at 32.
#[inline]
pub unsafe fn set_multiple_low1(mut bitmask: u32) {
    // Pins 34+ are input-only
    bitmask &= 0x3;

    GPIO.out1_w1tc.val = bitmask;
}
