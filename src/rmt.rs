//! Remote Control (RMT) module driver.
//!
//! The RMT (Remote Control) module driver can be used to send and receive infrared remote control
//! signals. Due to flexibility of RMT module, the driver can also be used to generate or receive
//! many other types of signals.
//!
//! This module is a wrapper around the [IDF RMT](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
//! implementation. It is recommended to read before using this module.
//!
//! This is an initial implementation supporting:
//!  * Transmission, currently only blocking.
//!
//! Not supported:
//! * Receiving.
//! * No channel locking. (To prevent users from accidentally reusing a channel.)
//! * No buffer protection while transmitting.
//! * Change of config after initialisation.
//!
//! # Example Usage
//!
//! TODO: Example!
//!
//! See the `examples/` folder of this repository for more.

// TODO: Do we need to prevent users from creating two drivers on the same channel?
// TODO: Should probably prevent writing to buffer while rmt is transmitting:
//       "We must make sure the item data will not be damaged when the driver is still sending items in driver interrupt."
use crate::gpio::OutputPin;
use embedded_hal::digital::PinState;
use esp_idf_sys::{
    esp, rmt_channel_t_RMT_CHANNEL_0, rmt_channel_t_RMT_CHANNEL_1, rmt_channel_t_RMT_CHANNEL_2,
    rmt_channel_t_RMT_CHANNEL_3, rmt_config, rmt_config_t, rmt_config_t__bindgen_ty_1,
    rmt_driver_install, rmt_get_counter_clock, rmt_item32_t, rmt_item32_t__bindgen_ty_1,
    rmt_item32_t__bindgen_ty_1__bindgen_ty_1, rmt_mode_t_RMT_MODE_RX, rmt_mode_t_RMT_MODE_TX,
    rmt_tx_config_t, rmt_write_items, EspError, RMT_CHANNEL_FLAGS_AWARE_DFS,
};
use std::mem::ManuallyDrop;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u32)]
pub enum Channel {
    // TODO: Work out the different number of channels per chip.
    Channel0 = rmt_channel_t_RMT_CHANNEL_0,
    Channel1 = rmt_channel_t_RMT_CHANNEL_1,
    Channel2 = rmt_channel_t_RMT_CHANNEL_2,
    Channel3 = rmt_channel_t_RMT_CHANNEL_3,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u32)]
pub enum Mode {
    Tx = rmt_mode_t_RMT_MODE_TX,
    Rx = rmt_mode_t_RMT_MODE_RX,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Pulse {
    ticks: PulseTicks,
    pin_state: PinState,
}

impl Pulse {
    pub fn new(pin_state: PinState, ticks: PulseTicks) -> Self {
        Pulse { pin_state, ticks }
    }
}

pub struct WriterConfig {
    config: rmt_config_t,
}

impl WriterConfig {
    pub fn new<OP>(pin: &OP, channel: Channel) -> Self
    where
        OP: OutputPin,
    {
        // Defaults from https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/rmt.h#L101
        Self {
            config: rmt_config_t {
                rmt_mode: rmt_mode_t_RMT_MODE_TX,
                channel: channel as u32,
                gpio_num: pin.pin(),
                clk_div: 80,
                mem_block_num: 1,
                flags: 0,
                __bindgen_anon_1: rmt_config_t__bindgen_ty_1 {
                    tx_config: rmt_tx_config_t {
                        carrier_freq_hz: 38000,
                        carrier_level: PinState::High as u32,
                        idle_level: PinState::Low as u32,
                        carrier_duty_percent: 33,
                        loop_count: 0,
                        carrier_en: false,
                        loop_en: false,
                        idle_output_en: true,
                    },
                },
            },
        }
    }

    /// Channel can work during APB clock scaling.
    ///
    /// When set, RMT channel will take REF_TICK or XTAL as source clock. The benefit is, RMT
    /// channel can continue work even when APB clock is changing.
    pub fn aware_dfs(mut self, enable: bool) -> Self {
        if enable {
            self.config.flags |= RMT_CHANNEL_FLAGS_AWARE_DFS;
        } else {
            self.config.flags &= !RMT_CHANNEL_FLAGS_AWARE_DFS;
        }
        self
    }

    pub fn mem_block_num(mut self, mem_block_num: u8) -> Self {
        self.config.mem_block_num = mem_block_num;
        self
    }

    pub fn clock_divider(mut self, divider: u8) -> Self {
        self.config.clk_div = divider;
        self
    }

    pub fn loop_enabled(mut self, enabled: bool) -> Self {
        self.config.__bindgen_anon_1.tx_config.loop_en = enabled;
        self
    }

    pub fn loop_count(mut self, count: u32) -> Self {
        self.config.__bindgen_anon_1.tx_config.loop_count = count;
        self
    }

    pub fn carrier_enabled(mut self, enabled: bool) -> Self {
        self.config.__bindgen_anon_1.tx_config.carrier_en = enabled;
        self
    }

    pub fn carrier_freq_hz(mut self, freq: u32) -> Self {
        self.config.__bindgen_anon_1.tx_config.carrier_freq_hz = freq;
        self
    }

    // TODO: Restrict 0-100 using a newtype.
    pub fn carrier_duty_percent(mut self, percent: u8) -> Self {
        self.config.__bindgen_anon_1.tx_config.carrier_duty_percent = percent;
        self
    }

    pub fn carrier_level(mut self, level: PinState) -> Self {
        self.config.__bindgen_anon_1.tx_config.carrier_level = level as u32;
        self
    }

    pub fn idle_level(mut self, level: PinState) -> Self {
        self.config.__bindgen_anon_1.tx_config.idle_level = level as u32;
        self
    }

    pub fn idle_output_enable(mut self, enabled: bool) -> Self {
        self.config.__bindgen_anon_1.tx_config.idle_output_en = enabled;
        self
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct PulseTicks(u16);

impl PulseTicks {
    const MAX: u16 = 32767;

    /// PulseTick needs to be unsigned 15 bits: 0-32767 inclusive.
    ///
    /// It will return `None` if it's out of range.
    pub fn new(v: u16) -> Option<Self> {
        if v > 32767 {
            // TODO: Not sure of a nicer way to deal with this? Allow an overflow like rust does?
            None
        } else {
            Some(Self(v))
        }
    }

    pub fn max() -> Self {
        Self(Self::MAX)
    }
}

pub struct Writer {
    config: rmt_config_t,

    /// This must be ManuallyDrop to ensure that it isn't automatically dropped before the driver is
    /// done using the items.
    // TODO: Manually drop this!
    items: ManuallyDrop<Vec<rmt_item32_t>>,

    half_inserted: bool,

    ticks_per_ns: Option<f32>,
}

impl Writer {
    pub fn new(config: WriterConfig) -> Result<Writer, EspError> {
        let s = Writer {
            config: config.config,
            items: Default::default(),
            half_inserted: false,
            ticks_per_ns: None,
        };

        unsafe {
            esp!(rmt_config(&s.config))?;
            esp!(rmt_driver_install(s.config.channel as u32, 0, 0))?;
        }

        Ok(s)
    }

    pub fn counter_clock(&self) -> Result<u32, EspError> {
        let mut ticks_hz: u32 = 0;
        esp!(unsafe { rmt_get_counter_clock(self.config.channel, &mut ticks_hz) })?;
        Ok(ticks_hz)
    }

    /// Returns None if the resulting ticks from the ns conversion is too high.
    /// See `PulseTicks` for details.
    pub fn pulse_ns(&mut self, pin_state: PinState, ns: u32) -> Result<Option<Pulse>, EspError> {
        let ticks_per_ns = match &self.ticks_per_ns {
            None => {
                let ticks_hz = self.counter_clock()?;
                let ticks_per_ns = ticks_hz as f32 / 1e9;
                self.ticks_per_ns = Some(ticks_per_ns);
                self.ticks_per_ns.as_ref().unwrap()
            }
            Some(t) => t,
        };
        let ticks = (ticks_per_ns * ns as f32) as u16;
        match PulseTicks::new(ticks) {
            Some(ticks) => Ok(Some(Pulse::new(pin_state, ticks))),
            None => Ok(None),
        }
    }

    pub fn add<I>(&mut self, pulses: I) -> Result<(), EspError>
    where
        I: IntoIterator<Item = Pulse>,
    {
        for pulse in pulses {
            println!("{:?} {}", &pulse.pin_state, pulse.ticks.0);

            let len = self.items.len();
            // TODO: Replace half_inserted with a reference to `Option<&Item>` to prevent
            // the unwrap below.
            if self.half_inserted {
                // This unwrap() shouldn't panic because len() will always be at least 1 when
                // something has been half_inserted.
                let item = self.items.get_mut(len - 1).unwrap();

                // SAFETY: We have retrieved this item which is previously populated with the same
                // union field.
                let inner = unsafe { &mut item.__bindgen_anon_1.__bindgen_anon_1 };

                inner.set_level1(pulse.pin_state as u32);
                inner.set_duration1(pulse.ticks.0 as u32);
            } else {
                let mut inner = rmt_item32_t__bindgen_ty_1__bindgen_ty_1::default();
                inner.set_level0(pulse.pin_state as u32);
                inner.set_duration0(pulse.ticks.0 as u32);
                let item = esp_idf_sys::rmt_item32_t {
                    __bindgen_anon_1: rmt_item32_t__bindgen_ty_1 {
                        __bindgen_anon_1: inner,
                    },
                };
                self.items.push(item);
            };

            self.half_inserted = !self.half_inserted;
        }

        Ok(())
    }

    pub fn clear(&mut self) -> Result<(), EspError> {
        self.items.clear();
        self.half_inserted = false;
        Ok(())
    }

    /// Start sending the pulses.
    pub fn start(&self) -> Result<(), EspError> {
        esp!(unsafe {
            rmt_write_items(
                self.config.channel as u32,
                self.items.as_ptr(),
                self.items.len() as i32,
                true, // TODO: Blocking.
            )
        })
    }

    pub fn stop(&self) -> Result<(), EspError> {
        todo!()
    }
}

impl Drop for Writer {
    fn drop(&mut self) {
        todo!("Ensure we have stopped before dropping items.");
    }
}
