use core::time::Duration;

use esp_idf_sys::{EspError, TickType_t, ESP_ERR_INVALID_ARG};

use crate::rmt::ClockSource;
use crate::units::{FromValueType, Hertz};

/// A percentage from 0 to 100%, used to specify the duty percentage in [`CarrierConfig`].
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct DutyPercent(f32);

impl DutyPercent {
    /// Must be between 0 and 1.0, otherwise an error is returned.
    pub fn new(value: f32) -> Result<Self, EspError> {
        if !(0.0..=1.0).contains(&value) {
            Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())
        } else {
            Ok(Self(value))
        }
    }

    /// Returns the percentage as a value between 0.0 and 1.0.
    #[must_use]
    pub const fn to_f32(&self) -> f32 {
        self.0
    }
}

/// Configuration setting for looping a signal.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Loop {
    /// No looping, transmit the signal only once.
    None,
    /// Repeats the transmission indefinitely.
    Endless,
    /// Repeats the transmission for the specified number of times.
    Count(u32),
}

/// Controls how the channel accesses memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryAccess {
    /// Enables the DMA backend for the channel. Using the DMA allows a significant amount of
    /// the channel's workload to be offloaded from the CPU. However, the DMA backend is not
    /// available on all ESP chips, please refer to [TRM] before you enable this option.
    /// Or you might encounter a `ESP_ERR_NOT_SUPPORTED` error.
    ///
    /// [TRM]: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#rmt
    Direct {
        /// This field controls the size of the internal DMA buffer. To achieve a better
        /// throughput and smaller CPU overhead, you can set a larger value, e.g., `1024`.
        memory_block_symbols: usize,
    },
    /// Disables direct memory access. The CPU will have to read/write the data transferred from/to the RMT memory.
    Indirect {
        /// This field controls the size of the dedicated memory block owned by the channel,
        /// which should be at least 64.
        memory_block_symbols: usize,
    },
}

impl MemoryAccess {
    #[must_use]
    pub(crate) fn is_direct(&self) -> bool {
        matches!(self, Self::Direct { .. })
    }

    #[must_use]
    pub(crate) fn symbols(&self) -> usize {
        let (Self::Direct {
            memory_block_symbols,
        }
        | Self::Indirect {
            memory_block_symbols,
        }) = self;
        *memory_block_symbols
    }
}

impl Default for MemoryAccess {
    fn default() -> Self {
        Self::Indirect {
            memory_block_symbols: 64,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TxChannelConfig {
    /// Selects the source clock for the RMT channel.
    ///
    /// Note that, the selected clock is also used by other channels,
    /// which means the user should ensure this configuration is the
    /// same when allocating other channels, regardless of TX or RX.
    /// For the effect on the power consumption of different clock sources,
    /// please refer to the [Power Management] section.
    ///
    /// [Power Management]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/rmt.html#rmt-power-management
    pub clock_source: ClockSource,
    /// Sets the resolution of the internal tick counter. The timing parameter
    /// of the RMT signal is calculated based on this **tick**.
    pub resolution: Hertz,
    /// Controls how the channel accesses memory.
    pub memory_access: MemoryAccess,
    /// Sets the depth of the internal transaction queue, the deeper the queue,
    /// the more transactions can be prepared in the backlog.
    pub transaction_queue_depth: usize,
    /// Set the priority of the interrupt. If set to 0, then the driver will use a
    /// interrupt with low or medium priority (priority level may be one of 1, 2 or 3),
    /// otherwise use the priority indicated by [`TxChannelConfig::interrupt_priority`].
    ///
    /// Please pay attention that once the interrupt priority is set, it cannot be changed
    /// until the channel is dropped.
    #[cfg(esp_idf_version_at_least_5_1_2)]
    pub interrupt_priority: i32,
    /// Is used to decide whether to invert the RMT signal before sending it to the GPIO pad.
    pub invert_out: bool,
    /// The signal output from the GPIO will be fed to the input path as well.
    pub io_loop_back: bool,
    /// Configure the GPIO as open-drain mode.
    pub io_od_mode: bool,
    /// Configures if the driver allows the system to power down the peripheral in light sleep mode.
    /// Before entering sleep, the system will backup the RMT register context, which will be restored
    /// later when the system exit the sleep mode. Powering down the peripheral can save more power,
    /// but at the cost of more memory consumed to save the register context. It's a tradeoff between
    /// power consumption and memory consumption. This configuration option relies on specific
    /// hardware feature, if you enable it on an unsupported chip, you will see error message
    /// like not able to power down in light sleep.
    #[cfg(esp_idf_version_at_least_5_4_0)]
    pub allow_pd: bool,
    // This field is intentionally hidden to prevent non-exhaustive pattern matching.
    // You should only construct this struct using the `..Default::default()` pattern.
    // If you use this field directly, your code might break in future versions.
    #[doc(hidden)]
    #[allow(dead_code)]
    pub __internal: (),
}

impl Default for TxChannelConfig {
    fn default() -> Self {
        Self {
            clock_source: Default::default(),
            resolution: 1.MHz().into(),
            memory_access: Default::default(),
            transaction_queue_depth: 4,
            #[cfg(esp_idf_version_at_least_5_1_2)]
            interrupt_priority: 0,
            invert_out: false,
            io_loop_back: false,
            io_od_mode: false,
            #[cfg(esp_idf_version_at_least_5_4_0)]
            allow_pd: false,
            __internal: (),
        }
    }
}

#[derive(Debug, Clone)]
pub struct TransmitConfig {
    /// Specify the times of transmission in a loop, -1 means transmitting in an infinite loop
    pub loop_count: Loop,
    /// Set the output level for the "End Of Transmission"
    pub eot_level: bool,
    /// If set, when the transaction queue is full, driver will not block the thread but return directly.
    #[cfg(esp_idf_version_at_least_5_1_3)]
    pub queue_non_blocking: bool,
    // This field is intentionally hidden to prevent non-exhaustive pattern matching.
    // You should only construct this struct using the `..Default::default()` pattern.
    // If you use this field directly, your code might break in future versions.
    #[doc(hidden)]
    #[allow(dead_code)]
    pub __internal: (),
}

impl Default for TransmitConfig {
    fn default() -> Self {
        Self {
            loop_count: Loop::None,
            eot_level: false,
            #[cfg(esp_idf_version_at_least_5_1_3)]
            queue_non_blocking: false,
            __internal: (),
        }
    }
}

#[derive(Debug, Clone)]
pub struct CarrierConfig {
    /// Carrier wave frequency, in Hz, 0 means disabling the carrier.
    pub frequency: Hertz,
    /// Carrier wave duty cycle (0~100%)
    pub duty_cycle: DutyPercent,
    /// Specify the polarity of carrier, by default it's modulated to base signal's high level
    pub polarity_active_low: bool,
    /// If set, the carrier can always exist even there's not transfer undergoing
    pub always_on: bool,
    // This field is intentionally hidden to prevent non-exhaustive pattern matching.
    // You should only construct this struct using the `..Default::default()` pattern.
    // If you use this field directly, your code might break in future versions.
    #[doc(hidden)]
    #[allow(dead_code)]
    pub __internal: (),
}

impl Default for CarrierConfig {
    fn default() -> Self {
        Self {
            frequency: 38000.Hz(),
            duty_cycle: DutyPercent(0.5),
            polarity_active_low: false,
            always_on: false,
            __internal: (),
        }
    }
}

#[derive(Debug, Clone)]
pub struct RxChannelConfig {
    /// Selects the source clock for the RMT channel.
    ///
    /// Note that, the selected clock is also used by other channels,
    /// which means the user should ensure this configuration is the
    /// same when allocating other channels, regardless of TX or RX.
    /// For the effect on the power consumption of different clock sources,
    /// please refer to the [Power Management] section.
    ///
    /// [Power Management]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/rmt.html#rmt-power-management
    pub clock_source: ClockSource,
    /// Sets the resolution of the internal tick counter. The timing parameter
    /// of the RMT signal is calculated based on this **tick**.
    pub resolution: Hertz,
    /// Controls how the channel accesses memory.
    pub memory_access: MemoryAccess,
    /// Set the priority of the interrupt. If set to 0, then the driver will use a
    /// interrupt with low or medium priority (priority level may be one of 1, 2 or 3),
    /// otherwise use the priority indicated by [`RxChannelConfig::interrupt_priority`].
    ///
    /// Please pay attention that once the interrupt priority is set, it cannot be changed
    /// until the channel is dropped.
    #[cfg(esp_idf_version_at_least_5_1_2)]
    pub interrupt_priority: i32,
    /// Is used to decide whether to invert the incoming RMT signal.
    pub invert_in: bool,
    /// The signal output from the GPIO will be fed to the input path as well.
    pub io_loop_back: bool,
    /// Configures if the driver allows the system to power down the peripheral in light sleep mode.
    /// Before entering sleep, the system will backup the RMT register context, which will be restored
    /// later when the system exit the sleep mode. Powering down the peripheral can save more power,
    /// but at the cost of more memory consumed to save the register context. It's a tradeoff between
    /// power consumption and memory consumption. This configuration option relies on specific
    /// hardware feature, if you enable it on an unsupported chip, you will see error message
    /// like not able to power down in light sleep.
    #[cfg(esp_idf_version_at_least_5_4_0)]
    pub allow_pd: bool,
    // This field is intentionally hidden to prevent non-exhaustive pattern matching.
    // You should only construct this struct using the `..Default::default()` pattern.
    // If you use this field directly, your code might break in future versions.
    #[doc(hidden)]
    #[allow(dead_code)]
    pub __internal: (),
}

impl Default for RxChannelConfig {
    fn default() -> Self {
        Self {
            clock_source: Default::default(),
            resolution: 1.MHz().into(),
            memory_access: Default::default(),
            #[cfg(esp_idf_version_at_least_5_1_2)]
            interrupt_priority: 0,
            invert_in: false,
            io_loop_back: false,
            #[cfg(esp_idf_version_at_least_5_4_0)]
            allow_pd: false,
            __internal: (),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ReceiveConfig {
    /// Specifies the minimal valid pulse duration in either high or low logic levels.
    ///
    /// A pulse width that is smaller than this value is treated as a glitch, and ignored by the hardware.
    pub signal_range_min: Duration,
    /// Specifies the maximum valid pulse duration in either high or low logic levels.
    ///
    /// A pulse width that is bigger than this value is treated as Stop Signal, and the
    /// receiver generates receive-complete event immediately.
    pub signal_range_max: Duration,
    /// Set this flag if the incoming data is very long, and the driver can only receive the data
    /// piece by piece, because the user buffer is not sufficient to save all the data.
    #[cfg(esp_idf_version_at_least_5_3_0)]
    pub enable_partial_rx: bool,
    /// Maximum time to wait for data to be received.
    ///
    /// If `None`, the driver will wait indefinitely.
    pub timeout: Option<TickType_t>,
    // This field is intentionally hidden to prevent non-exhaustive pattern matching.
    // You should only construct this struct using the `..Default::default()` pattern.
    // If you use this field directly, your code might break in future versions.
    #[doc(hidden)]
    #[allow(dead_code)]
    pub __internal: (),
}

impl Default for ReceiveConfig {
    fn default() -> Self {
        Self {
            signal_range_min: Duration::ZERO,
            // This is the maximum duration that the driver supports.
            signal_range_max: Duration::from_nanos(u32::MAX as u64),
            #[cfg(esp_idf_version_at_least_5_3_0)]
            enable_partial_rx: false,
            timeout: None,
            __internal: (),
        }
    }
}
