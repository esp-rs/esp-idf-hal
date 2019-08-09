extern crate alloc;

use crate::{
    delay::portMAX_DELAY,
    errors::{Error, EspError, Result},
};
use alloc::{boxed::Box, vec, vec::Vec};
use core::{convert::TryInto, mem::ManuallyDrop};
use embedded_hal::digital::v2::OutputPin;
use esp_idf_sys::{
    rmt_config, rmt_config_t, rmt_config_t__bindgen_ty_1, rmt_driver_install, rmt_driver_uninstall,
    rmt_item32_t, rmt_item32_t__bindgen_ty_1, rmt_item32_t__bindgen_ty_1__bindgen_ty_1,
    rmt_mode_t_RMT_MODE_TX, rmt_set_idle_level, rmt_tx_config_t, rmt_wait_tx_done, rmt_write_items,
    TickType_t, ESP_OK,
};

#[repr(u32)]
#[derive(Clone, Copy, Debug)]
pub enum Which {
    Channel0,
    Channel1,
    Channel2,
    Channel3,
    Channel4,
    Channel5,
    Channel6,
    Channel7,
}

#[repr(u32)]
#[derive(Clone, Copy, Debug)]
pub enum Level {
    Low,
    High,
}

#[derive(Clone, Debug)]
pub struct CarrierConfig {
    pub freq_hz: u32,
    pub duty_percent: u8,
    pub level: Level,
}

#[derive(Clone, Copy, Debug)]
pub enum IdleState {
    OutputDisabled,
    OutputEnabled(Level),
}

impl IdleState {
    fn level(&self) -> Level {
        use IdleState::*;

        match self {
            OutputDisabled => Level::Low,
            OutputEnabled(level) => *level,
        }
    }

    fn output_enabled(&self) -> bool {
        if let IdleState::OutputEnabled(..) = self {
            true
        } else {
            false
        }
    }
}

#[derive(Clone, Debug)]
pub struct TxConfig {
    pub loop_enable: bool,
    pub carrier: Option<CarrierConfig>,
    pub idle_state: IdleState,
}

impl From<TxConfig> for rmt_config_t__bindgen_ty_1 {
    fn from(value: TxConfig) -> Self {
        let TxConfig {
            loop_enable,
            carrier,
            idle_state,
        } = value;
        let carrier_en = carrier.is_some();
        // Use 0 values if disabled
        let carrier = carrier.unwrap_or(CarrierConfig {
            freq_hz: 0,
            duty_percent: 0,
            level: Level::Low,
        });

        rmt_config_t__bindgen_ty_1 {
            tx_config: rmt_tx_config_t {
                loop_en: loop_enable,
                carrier_freq_hz: carrier.freq_hz,
                carrier_duty_percent: carrier.duty_percent,
                carrier_level: carrier.level as u32,
                carrier_en,
                idle_level: idle_state.level() as u32,
                idle_output_en: idle_state.output_enabled(),
            },
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Item {
    pub duration: u16,
    pub level: Level,
}

const END_ITEM: Item = Item {
    duration: 0,
    level: Level::Low,
};

struct Driver {
    which: Which,
}

impl Driver {
    unsafe fn new(
        which: Which,
        config: &rmt_config_t,
        rx_ring_buffer_size: usize,
    ) -> Result<Driver> {
        EspError(rmt_config(config)).into_result()?;

        // TODO: interrupt flags
        EspError(rmt_driver_install(which as u32, rx_ring_buffer_size, 0)).into_result()?;

        Ok(Driver { which })
    }
}

impl Drop for Driver {
    fn drop(&mut self) {
        let err = unsafe { rmt_driver_uninstall(self.which as u32) };
        assert_eq!(err, ESP_OK as i32);
    }
}

pub struct TxChannel {
    driver: Driver,
    /// This must be ManuallyDrop to ensure that it isn't automatically dropped before the driver is
    /// done using the items.
    tx_items: Option<ManuallyDrop<Box<[rmt_item32_t]>>>,
}

impl TxChannel {
    /// clk_div divides the 80MHz APB clock, e.g. for 1Mhz use clk_div = 80.
    /// The 1MHz REF tick clock supported by the RMT peripheral is currently unsupported.
    pub unsafe fn new(which: Which, gpio_num: u8, clk_div: u8, config: TxConfig) -> Result<Self> {
        let config = rmt_config_t {
            rmt_mode: rmt_mode_t_RMT_MODE_TX,
            channel: which as u32,
            clk_div,
            gpio_num: gpio_num.into(),
            // Just a single 64*32-bit block. TODO: support other values
            mem_block_num: 1,
            __bindgen_anon_1: config.into(),
        };

        let driver = Driver::new(which, &config, 0)?;

        Ok(Self {
            driver,
            tx_items: None,
        })
    }

    fn free_items(&mut self) {
        if let Some(tx_items) = self.tx_items.take() {
            // Drop the items
            ManuallyDrop::into_inner(tx_items);
        }
    }

    pub fn wait(&mut self, timeout: Option<TickType_t>) -> Result<()> {
        let wait_time = timeout.unwrap_or(portMAX_DELAY);

        unsafe {
            EspError(rmt_wait_tx_done(self.driver.which as u32, wait_time)).into_result()?;
        }

        // Done waiting => We can free the items
        self.free_items();

        Ok(())
    }

    /// Busy wait
    pub fn wait_busy(&mut self) -> Result<()> {
        loop {
            let result =
                unsafe { EspError(rmt_wait_tx_done(self.driver.which as u32, 0)).into_result() };

            match result {
                Ok(()) => {
                    self.free_items();
                    break Ok(());
                }

                Err(Error::Timeout) => {
                    continue;
                }

                _ => {
                    break result;
                }
            }
        }
    }

    fn convert_items<I>(items: I) -> Vec<rmt_item32_t>
    where
        I: IntoIterator<Item = Item>,
    {
        let mut items = items.into_iter().fuse();

        let mut v = vec![];

        loop {
            let item1 = items.next();
            let item2 = items.next();
            let done = item1.is_none() || item2.is_none();

            let item1 = item1.unwrap_or(END_ITEM);
            let item2 = item2.unwrap_or(END_ITEM);

            let rmt_item32 = rmt_item32_t {
                __bindgen_anon_1: rmt_item32_t__bindgen_ty_1 {
                    __bindgen_anon_1: rmt_item32_t__bindgen_ty_1__bindgen_ty_1 {
                        _bitfield_1: rmt_item32_t__bindgen_ty_1__bindgen_ty_1::new_bitfield_1(
                            item1.duration.into(),
                            item1.level as u32,
                            item2.duration.into(),
                            item2.level as u32,
                        ),
                    },
                },
            };

            v.push(rmt_item32);

            if done {
                break;
            }
        }

        v
    }

    /// Starts transmitting items. Call wait() to wait for the transmission to end. If a previous
    /// transmission is active, waits for it to end before starting a new transmission.
    pub fn write<I>(&mut self, items: I) -> Result<()>
    where
        I: IntoIterator<Item = Item>,
    {
        if self.tx_items.is_some() {
            self.wait(None)?;
        }

        let items = Self::convert_items(items);
        let items = items.into_boxed_slice();

        let num_items = items.len();
        let items_ptr = items.as_ptr();

        // Store the items in our ManuallyDrop before passing to driver
        debug_assert!(self.tx_items.is_none());
        self.tx_items = Some(ManuallyDrop::new(items));

        unsafe {
            EspError(rmt_write_items(
                self.driver.which as u32,
                items_ptr,
                num_items.try_into().unwrap(),
                false,
            ))
            .into_result()?;
        }

        Ok(())
    }

    pub fn set_idle_state(&mut self, idle_state: IdleState) -> Result<()> {
        unsafe {
            EspError(rmt_set_idle_level(
                self.driver.which as u32,
                idle_state.output_enabled(),
                idle_state.level() as u32,
            ))
            .into_result()
        }
    }
}

impl Drop for TxChannel {
    fn drop(&mut self) {
        if self.tx_items.is_some() {
            // This also frees the items
            self.wait(None).unwrap();
        }
    }
}

impl OutputPin for TxChannel {
    type Error = Error;

    fn set_low(&mut self) -> Result<()> {
        self.wait(None)?;
        self.set_idle_state(IdleState::OutputEnabled(Level::Low))?;
        Ok(())
    }

    fn set_high(&mut self) -> Result<()> {
        self.wait(None)?;
        self.set_idle_state(IdleState::OutputEnabled(Level::High))?;
        Ok(())
    }
}
