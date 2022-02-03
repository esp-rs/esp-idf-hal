use crate::gpio::OutputPin;
use esp_idf_sys::{
    esp, rmt_channel_t_RMT_CHANNEL_0, rmt_channel_t_RMT_CHANNEL_1, rmt_channel_t_RMT_CHANNEL_2,
    rmt_channel_t_RMT_CHANNEL_3, rmt_config, rmt_config_t, rmt_config_t__bindgen_ty_1,
    rmt_driver_install, rmt_get_counter_clock, rmt_item32_t, rmt_item32_t__bindgen_ty_1,
    rmt_item32_t__bindgen_ty_1__bindgen_ty_1, rmt_mode_t_RMT_MODE_RX, rmt_mode_t_RMT_MODE_TX,
    rmt_tx_config_t, rmt_write_items, EspError,
};
use std::mem::ManuallyDrop;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u32)]
pub enum Channel {
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
#[repr(u32)]
pub enum Level {
    Low,
    High,
}

impl Default for Level {
    fn default() -> Self {
        Self::Low
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PulseDuration {
    Tick(u32),
    Nanos(u32),
}

impl Default for PulseDuration {
    fn default() -> Self {
        Self::Tick(0)
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Default)]
pub struct Pulse {
    duration: PulseDuration,
    level: Level,
}

impl Pulse {
    pub fn new(level: Level, duration: PulseDuration) -> Self {
        Pulse { level, duration }
    }
}

pub struct WritePulsesBuilder {
    config: rmt_config_t,
}

impl WritePulsesBuilder {
    pub fn new(pin: &dyn OutputPin<Error = EspError>, channel: Channel) -> Self {
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
                        carrier_level: Level::High as u32,
                        idle_level: Level::Low as u32,
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

    pub fn build(self) -> Result<WritePulses, EspError> {
        unsafe {
            esp!(rmt_config(&self.config))?;
            esp!(rmt_driver_install(self.config.channel as u32, 0, 0))?;
        }

        Ok(WritePulses {
            config: self.config, // TODO: Pass in reference?
            items: Default::default(),
            half_inserted: false,
        })
    }

    pub fn set_clock_divider(mut self, divider: u8) -> Self {
        self.config.clk_div = divider;
        self
    }

    pub fn set_loop_enabled(mut self, enabled: bool) -> Self {
        self.config.__bindgen_anon_1.tx_config.loop_en = enabled;
        self
    }

    pub fn set_carrier_enabled(mut self, enabled: bool) -> Self {
        self.config.__bindgen_anon_1.tx_config.carrier_en = enabled;
        self
    }

    pub fn set_carrier_freq_hz(mut self, freq: u32) -> Self {
        self.config.__bindgen_anon_1.tx_config.carrier_freq_hz = freq;
        self
    }
}

pub struct WritePulses {
    config: rmt_config_t,

    /// This must be ManuallyDrop to ensure that it isn't automatically dropped before the driver is
    /// done using the items.
    // TODO: Manually drop this!
    items: ManuallyDrop<Vec<rmt_item32_t>>,

    half_inserted: bool,
}

impl WritePulses {
    pub fn add<I>(&mut self, pulses: I) -> Result<(), EspError>
    where
        I: IntoIterator<Item = Pulse>,
    {
        let mut ticks_hz: u32 = 0;
        esp!(unsafe { rmt_get_counter_clock(self.config.channel, &mut ticks_hz) })?;

        for pulse in pulses {
            let ticks = match pulse.duration {
                PulseDuration::Tick(ticks) => ticks,
                PulseDuration::Nanos(ns) => {
                    let ticks_per_ns = ticks_hz as f32 / 1e9;
                    let ticks = ticks_per_ns * ns as f32;
                    ticks as u32
                }
            };

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

                inner.set_level1(pulse.level as u32);
                inner.set_duration1(ticks as u32);
            } else {
                let mut inner = rmt_item32_t__bindgen_ty_1__bindgen_ty_1::default();
                inner.set_level0(pulse.level as u32);
                inner.set_duration0(ticks as u32);
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

    pub fn clear(&mut self) {
        self.items.clear();
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

    pub fn stop(&self) {
        todo!()
    }
}
