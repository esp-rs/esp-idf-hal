//! CAN bus peripheral control.
//!
//! It is called Two-Wire Automotive Interface (TWAI) in ESP32 documentation.
//!
//! # Example
//!
//! Create a CAN peripheral and then transmit and receive a message.
//! ```
//! use embedded_can::nb::Can;
//! use embedded_can::Frame;
//! use embedded_can::StandardId;
//! use esp_idf_hal::prelude::*;
//! use esp_idf_hal::can;
//!
//! let peripherals = Peripherals::take().unwrap();
//! let pins = peripherals.pins;
//!
//! // filter to accept only CAN ID 881
//! let filter = can::config::Filter::Standard {filter: 881, mask: 0x7FF };
//! // filter that accepts all CAN IDs
//! // let filter = can::config::Filter::standard_allow_all();
//!
//! let timing = can::config::Timing::B500K;
//! let config = can::config::Config::new().filter(filter).timing(timing);
//! let mut can = can::CanDriver::new(peripherals.can, pins.gpio5, pins.gpio4, &config).unwrap();
//!
//! let tx_frame = can::Frame::new(StandardId::new(0x042).unwrap(), &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
//! nb::block!(can.transmit(&tx_frame)).unwrap();
//!
//! if let Ok(rx_frame) = nb::block!(can.receive()) {
//!    info!("rx {:}:", rx_frame);
//! }
//! ```

use esp_idf_sys::*;

use crate::delay::{BLOCK, NON_BLOCK};
use crate::gpio::*;
use crate::interrupt::IntrFlags;
use crate::peripheral::{Peripheral, PeripheralRef};

crate::embedded_hal_error!(CanError, embedded_can::Error, embedded_can::ErrorKind);

crate::embedded_hal_error!(
    Can02Error,
    embedded_hal_0_2::can::Error,
    embedded_hal_0_2::can::ErrorKind
);

pub type CanConfig = config::Config;

pub mod config {
    use enumset::EnumSet;
    use esp_idf_sys::*;

    use crate::interrupt::IntrFlags;

    /// CAN timing
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub enum Timing {
        B25K,
        B50K,
        B100K,
        B125K,
        B250K,
        B500K,
        B800K,
        B1M,
        Custom {
            baudrate_prescaler: u32,
            timing_segment_1: u8,
            timing_segment_2: u8,
            synchronization_jump_width: u8,
            triple_sampling: bool,
        },
    }

    impl From<Timing> for twai_timing_config_t {
        #[allow(clippy::needless_update)]
        fn from(resolution: Timing) -> Self {
            match resolution {
                Timing::B25K => twai_timing_config_t {
                    brp: 128,
                    tseg_1: 16,
                    tseg_2: 8,
                    sjw: 3,
                    triple_sampling: false,
                    ..Default::default()
                },
                Timing::B50K => twai_timing_config_t {
                    brp: 80,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                    ..Default::default()
                },
                Timing::B100K => twai_timing_config_t {
                    brp: 40,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                    ..Default::default()
                },
                Timing::B125K => twai_timing_config_t {
                    brp: 32,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                    ..Default::default()
                },
                Timing::B250K => twai_timing_config_t {
                    brp: 16,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                    ..Default::default()
                },
                Timing::B500K => twai_timing_config_t {
                    brp: 8,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                    ..Default::default()
                },
                Timing::B800K => twai_timing_config_t {
                    brp: 4,
                    tseg_1: 16,
                    tseg_2: 8,
                    sjw: 3,
                    triple_sampling: false,
                    ..Default::default()
                },
                Timing::B1M => twai_timing_config_t {
                    brp: 4,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                    ..Default::default()
                },
                Timing::Custom {
                    baudrate_prescaler,
                    timing_segment_1,
                    timing_segment_2,
                    synchronization_jump_width,
                    triple_sampling,
                } => twai_timing_config_t {
                    brp: baudrate_prescaler,
                    tseg_1: timing_segment_1,
                    tseg_2: timing_segment_2,
                    sjw: synchronization_jump_width,
                    triple_sampling,
                    ..Default::default()
                },
            }
        }
    }

    impl Default for Timing {
        fn default() -> Self {
            Self::B500K
        }
    }

    /// Is used to filter out unwanted CAN IDs (messages).
    ///
    /// Notice that Espressif TWAI (CAN in rest of the world) acceptance filtering
    /// works differently than common CAN filtering (for example mask bits are inversed).
    /// However here those differences are hidden away from the user and common CAN filtering is used.
    ///
    /// `mask` is used to determine which bits in the incoming CAN ID are compared with the `filter` value.
    /// Bits in `mask` mean:
    /// `0`: do not care - the bit is not used for the comparison
    /// `1`: must match - the bit of the incoming CAN ID must have the same state as in `filter`
    ///
    /// Notice that if `mask` is `0`, all CAN IDs are accepted regardless of `filter` value.
    ///
    /// ## Examples
    ///
    /// This shows how 11 bit CAN ID `0x3AA` goes through filtering engine and is finally accepted:
    /// ```
    /// // incoming id [ 0 1 1 1 0 1 0 1 0 1 0 ]
    /// // mask        [ 1 0 1 0 0 1 1 1 0 0 0 ]
    /// //               1 = compare
    /// //               0 = do not care
    /// // masked id   [ 0 _ 1 _ _ 1 0 1 _ _ _ ]
    /// // filter      [ 0 0 1 1 1 1 0 1 0 1 1 ]
    ///
    /// // incoming id [ 0 1 1 1 0 1 0 1 0 1 0 ]
    /// // accepted
    /// ```
    ///
    /// Notice that for example `0x7AA` would not be accepted because its MSB bit is `1`,
    /// but `filter` only accepts `0` in this bit position and `mask` says that this bit must be compared.
    ///
    /// Accept only CAN ID `0x567`
    /// ```
    /// let filter = 0x567;
    /// // every bit must match filter
    /// let mask   = 0x7FF;
    /// let f = Filter::Standard { filter, mask };
    /// ```
    ///
    /// Accept CAN IDs `0x560 - 0x56F`
    /// ```
    /// let filter = 0x560;
    /// // do not care about 4 LSB bits
    /// let mask   = 0x7F0;
    /// let f = Filter::Standard { filter, mask };
    /// ```
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub enum Filter {
        // Filter for 11 bit standard CAN IDs
        Standard { filter: u16, mask: u16 },
        // Filter for 29 bit extended CAN IDs
        Extended { filter: u32, mask: u32 },
    }

    impl Filter {
        /// Filter that allows all standard CAN IDs.
        pub fn standard_allow_all() -> Self {
            Self::Standard { filter: 0, mask: 0 }
        }

        /// Filter that accepts all extended CAN IDs.
        pub fn extended_allow_all() -> Self {
            Self::Extended { filter: 0, mask: 0 }
        }
    }

    impl Default for Filter {
        fn default() -> Self {
            Filter::standard_allow_all()
        }
    }

    #[derive(Debug, Copy, Clone, Default)]
    pub enum Mode {
        #[default]
        Normal,
        NoAck,
        ListenOnly,
    }

    impl From<Mode> for twai_mode_t {
        fn from(val: Mode) -> Self {
            match val {
                Mode::Normal => twai_mode_t_TWAI_MODE_NORMAL,
                Mode::NoAck => twai_mode_t_TWAI_MODE_NO_ACK,
                Mode::ListenOnly => twai_mode_t_TWAI_MODE_LISTEN_ONLY,
            }
        }
    }

    #[derive(Debug, Copy, Clone)]
    pub struct Alerts {
        value: u32,
    }

    impl Alerts {
        const TWAI_ALERT_TX_SUCCESS: u32 = 0x00000002;
        const TWAI_ALERT_TX_IDLE: u32 = 0x00000001;
        const TWAI_ALERT_RX_DATA: u32 = 0x00000004;
        const TWAI_ALERT_BELOW_ERR_WARN: u32 = 0x00000008;
        const TWAI_ALERT_ERR_ACTIVE: u32 = 0x00000010;
        const TWAI_ALERT_RECOVERY_IN_PROGRESS: u32 = 0x00000020;
        const TWAI_ALERT_BUS_RECOVERED: u32 = 0x00000040;
        const TWAI_ALERT_ARB_LOST: u32 = 0x00000080;
        const TWAI_ALERT_ABOVE_ERR_WARN: u32 = 0x00000100;
        const TWAI_ALERT_BUS_ERROR: u32 = 0x00000200;
        const TWAI_ALERT_TX_FAILED: u32 = 0x00000400;
        const TWAI_ALERT_RX_QUEUE_FULL: u32 = 0x00000800;
        const TWAI_ALERT_ERR_PASS: u32 = 0x00001000;
        const TWAI_ALERT_BUS_OFF: u32 = 0x00002000;
        const TWAI_ALERT_RX_FIFO_OVERRUN: u32 = 0x00004000;
        const TWAI_ALERT_TX_RETRIED: u32 = 0x00008000;
        const TWAI_ALERT_PERIPH_RESET: u32 = 0x00010000;
        const TWAI_ALERT_ALL: u32 = 0x0001FFFF;
        const TWAI_ALERT_NONE: u32 = 0x00000000;
        const TWAI_ALERT_AND_LOG: u32 = 0x00020000;

        /// Create new `Alerts` with no alerts set.
        pub const fn new() -> Self {
            Self {
                value: Self::TWAI_ALERT_NONE,
            }
        }

        /// Enable the `TX_SUCCESS` alert: The previous transmission was successful.
        pub fn tx_success(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_TX_SUCCESS,
            }
        }

        /// Enable the `TX_IDLE` alert: No more messages to transmit.
        pub fn tx_idle(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_TX_IDLE,
            }
        }

        /// Enable the `RX_DATA` alert: A frame has been received and added to the RX queue.
        pub fn rx_data(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_RX_DATA,
            }
        }

        /// Enable the `BELOW_ERR_WARN` alert: Both error counters have dropped below error warning limit.
        pub fn below_err_warn(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_BELOW_ERR_WARN,
            }
        }

        /// Enable the `ERR_ACTIVE` alert: TWAI controller has become error active.
        pub fn err_active(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_ERR_ACTIVE,
            }
        }

        /// Enable the `RECOVERY_IN_PROGRESS` alert: TWAI controller is undergoing bus recovery.
        pub fn recovery_in_progress(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_RECOVERY_IN_PROGRESS,
            }
        }

        /// Enable the `BUS_RECOVERED` alert: TWAI controller has successfully completed bus recovery.
        pub fn bus_recovered(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_BUS_RECOVERED,
            }
        }

        /// Enable the `ARB_LOST` alert: The previous transmission lost arbitration.
        pub fn arb_lost(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_ARB_LOST,
            }
        }

        /// Enable the `ABOVE_ERR_WARN` alert: One of the error counters have exceeded the error warning limit.
        pub fn above_err_warn(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_ABOVE_ERR_WARN,
            }
        }

        /// Enable the `BUS_ERROR` alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.
        pub fn bus_error(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_BUS_ERROR,
            }
        }

        /// Enable the `TX_FAILED` alert: The previous transmission has failed (for single shot transmission).
        pub fn tx_failed(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_TX_FAILED,
            }
        }

        /// Enable the `RX_QUEUE_FULL` alert: The RX queue is full causing a frame to be lost.
        pub fn rx_queue_full(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_RX_QUEUE_FULL,
            }
        }

        /// Enable the `ERR_PASS` alert: TWAI controller has become error passive.
        pub fn err_pass(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_ERR_PASS,
            }
        }

        /// Enable the `BUS_OFF` alert: Bus-off condition occurred. TWAI controller can no longer influence bus.
        pub fn bus_off(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_BUS_OFF,
            }
        }

        /// Enable the `RX_FIFO_OVERRUN` alert: An RX FIFO overrun has occurred.
        pub fn rx_fifo_overrun(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_RX_FIFO_OVERRUN,
            }
        }

        /// Enable the `TX_RETRIED` alert: An message transmission was cancelled and retried due to an errata workaround.
        pub fn tx_retried(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_TX_RETRIED,
            }
        }

        /// Enable the `PERIPH_RESET` alert: The TWAI controller was reset.
        pub fn periph_reset(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_PERIPH_RESET,
            }
        }

        /// Enable all alerts.
        pub fn all(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_ALL,
            }
        }

        /// Enable alert logging when they occur. Note that logging from the ISR is disabled if CONFIG_TWAI_ISR_IN_IRAM is enabled.
        pub fn and_log(self) -> Self {
            Self {
                value: self.value | Self::TWAI_ALERT_AND_LOG,
            }
        }
    }

    impl From<Alerts> for u32 {
        fn from(val: Alerts) -> Self {
            val.value
        }
    }

    impl Default for Alerts {
        fn default() -> Self {
            Self::new()
        }
    }

    #[derive(Debug, Clone)]
    pub struct Config {
        pub timing: Timing,
        pub filter: Filter,
        pub tx_queue_len: u32,
        pub rx_queue_len: u32,
        pub mode: Mode,
        pub alerts: Alerts,
        pub intr_flags: EnumSet<IntrFlags>,
    }

    impl Config {
        pub fn new() -> Self {
            Self {
                timing: Default::default(),
                filter: Default::default(),
                tx_queue_len: 5,
                rx_queue_len: 5,
                mode: Default::default(),
                alerts: Default::default(),
                intr_flags: EnumSet::<IntrFlags>::empty(),
            }
        }

        #[must_use]
        pub fn timing(mut self, timing: Timing) -> Self {
            self.timing = timing;
            self
        }

        #[must_use]
        pub fn filter(mut self, filter: Filter) -> Self {
            self.filter = filter;
            self
        }

        #[must_use]
        pub fn tx_queue_len(mut self, tx_queue_len: u32) -> Self {
            self.tx_queue_len = tx_queue_len;
            self
        }

        #[must_use]
        pub fn rx_queue_len(mut self, rx_queue_len: u32) -> Self {
            self.rx_queue_len = rx_queue_len;
            self
        }

        #[must_use]
        pub fn mode(mut self, mode: Mode) -> Self {
            self.mode = mode;
            self
        }

        #[must_use]
        pub fn alerts(mut self, alerts: Alerts) -> Self {
            self.alerts = alerts;
            self
        }

        #[must_use]
        pub fn intr_flags(mut self, flags: EnumSet<IntrFlags>) -> Self {
            self.intr_flags = flags;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self::new()
        }
    }
}

/// CAN abstraction
pub struct CanDriver<'d>(PeripheralRef<'d, CAN>);

unsafe impl<'d> Send for CanDriver<'d> {}

impl<'d> CanDriver<'d> {
    pub fn new(
        can: impl Peripheral<P = CAN> + 'd,
        tx: impl Peripheral<P = impl OutputPin> + 'd,
        rx: impl Peripheral<P = impl OutputPin> + 'd,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        crate::into_ref!(can, tx, rx);

        let general_config = twai_general_config_t {
            mode: config.mode.into(),
            tx_io: tx.pin(),
            rx_io: rx.pin(),
            clkout_io: -1,
            bus_off_io: -1,
            tx_queue_len: config.tx_queue_len,
            rx_queue_len: config.rx_queue_len,
            alerts_enabled: config.alerts.into(),
            clkout_divider: 0,
            intr_flags: IntrFlags::to_native(config.intr_flags) as _,
        };

        let timing_config = config.timing.into();

        // modify filter and mask to be compatible with TWAI acceptance filter
        let (filter, mask) = match config.filter {
            config::Filter::Standard { filter, mask } => {
                ((filter as u32) << 21, !((mask as u32) << 21))
            }
            config::Filter::Extended { filter, mask } => (filter << 3, !(mask << 3)),
        };

        let filter_config = twai_filter_config_t {
            acceptance_code: filter,
            acceptance_mask: mask,
            single_filter: true,
        };

        esp!(unsafe { twai_driver_install(&general_config, &timing_config, &filter_config) })?;
        esp!(unsafe { twai_start() })?;

        Ok(Self(can))
    }

    pub fn transmit(&mut self, frame: &Frame, timeout: TickType_t) -> Result<(), EspError> {
        esp!(unsafe { twai_transmit(&frame.0, timeout) })
    }

    pub fn receive(&mut self, timeout: TickType_t) -> Result<Frame, EspError> {
        let mut rx_msg = Default::default();

        match esp_result!(unsafe { twai_receive(&mut rx_msg, timeout) }, ()) {
            Ok(_) => Ok(Frame(rx_msg)),
            Err(err) => Err(err),
        }
    }
}

impl<'d> Drop for CanDriver<'d> {
    fn drop(&mut self) {
        esp!(unsafe { twai_stop() }).unwrap();
        esp!(unsafe { twai_driver_uninstall() }).unwrap();
    }
}

impl<'d> embedded_hal_0_2::blocking::can::Can for CanDriver<'d> {
    type Frame = Frame;
    type Error = Can02Error;

    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        self.transmit(frame, BLOCK).map_err(Can02Error::other)
    }

    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        self.receive(BLOCK).map_err(Can02Error::other)
    }
}

impl<'d> embedded_can::blocking::Can for CanDriver<'d> {
    type Frame = Frame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        self.transmit(frame, BLOCK).map_err(CanError::other)
    }

    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        self.receive(BLOCK).map_err(CanError::other)
    }
}

impl<'d> embedded_hal_0_2::can::nb::Can for CanDriver<'d> {
    type Frame = Frame;
    type Error = Can02Error;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        match self.transmit(frame, NON_BLOCK) {
            Ok(_) => Ok(None),
            Err(e) if e.code() == ESP_FAIL => Err(nb::Error::WouldBlock),
            Err(e) if e.code() == ESP_ERR_TIMEOUT => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(Can02Error::other(e))),
        }
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        match self.receive(NON_BLOCK) {
            Ok(frame) => Ok(frame),
            Err(e) if e.code() == ESP_ERR_TIMEOUT => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(Can02Error::other(e))),
        }
    }
}

impl<'d> embedded_can::nb::Can for CanDriver<'d> {
    type Frame = Frame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        match self.transmit(frame, NON_BLOCK) {
            Ok(_) => Ok(None),
            Err(e) if e.code() == ESP_FAIL => Err(nb::Error::WouldBlock),
            Err(e) if e.code() == ESP_ERR_TIMEOUT => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(CanError::other(e))),
        }
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        match self.receive(NON_BLOCK) {
            Ok(frame) => Ok(frame),
            Err(e) if e.code() == ESP_ERR_TIMEOUT => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(CanError::other(e))),
        }
    }
}

pub struct Frame(twai_message_t);

impl Frame {
    pub fn new(id: u32, extended: bool, data: &[u8]) -> Option<Self> {
        let dlc = data.len();

        if dlc <= 8 {
            // unions are not very well supported in rust
            // therefore setting those union flags is quite hairy
            let mut flags = twai_message_t__bindgen_ty_1::default();

            // set bits in an union
            unsafe { flags.__bindgen_anon_1.set_ss(1) };
            if extended {
                unsafe { flags.__bindgen_anon_1.set_extd(1) };
            }

            let mut payload = [0; 8];
            payload[..dlc].copy_from_slice(data);

            let twai_message = twai_message_t {
                __bindgen_anon_1: flags,
                identifier: id,
                data_length_code: dlc as u8,
                data: payload,
            };

            Some(Frame(twai_message))
        } else {
            None
        }
    }

    pub fn new_remote(id: u32, extended: bool, dlc: usize) -> Option<Self> {
        if dlc <= 8 {
            // unions are not very well supported in rust
            // therefore setting those union flags is quite hairy
            let mut flags = twai_message_t__bindgen_ty_1::default();

            // set bits in an union
            unsafe { flags.__bindgen_anon_1.set_rtr(1) };
            unsafe { flags.__bindgen_anon_1.set_ss(1) };
            if extended {
                unsafe { flags.__bindgen_anon_1.set_extd(1) };
            }

            let twai_message = twai_message_t {
                __bindgen_anon_1: flags,
                identifier: id,
                data_length_code: dlc as u8,
                data: [0; 8],
            };

            Some(Frame(twai_message))
        } else {
            None
        }
    }

    pub fn is_extended(&self) -> bool {
        unsafe { self.0.__bindgen_anon_1.__bindgen_anon_1.extd() == 1 }
    }

    pub fn is_remote_frame(&self) -> bool {
        unsafe { self.0.__bindgen_anon_1.__bindgen_anon_1.rtr() == 1 }
    }

    pub fn identifier(&self) -> u32 {
        self.0.identifier
    }

    pub fn dlc(&self) -> usize {
        self.0.data_length_code as usize
    }

    pub fn data(&self) -> &[u8] {
        &self.0.data[..self.dlc()]
    }
}

impl core::fmt::Display for Frame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "Frame {{ id: {}, remote: {}, data: {:?} }}",
            self.identifier(),
            self.is_remote_frame(),
            self.data()
        )
    }
}

impl embedded_hal_0_2::can::Frame for Frame {
    fn new(id: impl Into<embedded_hal_0_2::can::Id>, data: &[u8]) -> Option<Self> {
        let (id, extended) = match id.into() {
            embedded_hal_0_2::can::Id::Standard(id) => (id.as_raw() as u32, false),
            embedded_hal_0_2::can::Id::Extended(id) => (id.as_raw(), true),
        };

        Self::new(id, extended, data)
    }

    fn new_remote(id: impl Into<embedded_hal_0_2::can::Id>, dlc: usize) -> Option<Self> {
        let (id, extended) = match id.into() {
            embedded_hal_0_2::can::Id::Standard(id) => (id.as_raw() as u32, false),
            embedded_hal_0_2::can::Id::Extended(id) => (id.as_raw(), true),
        };

        Self::new_remote(id, extended, dlc)
    }

    fn is_extended(&self) -> bool {
        Frame::is_extended(self)
    }

    fn is_standard(&self) -> bool {
        !self.is_extended()
    }

    fn is_remote_frame(&self) -> bool {
        Frame::is_remote_frame(self)
    }

    fn is_data_frame(&self) -> bool {
        !self.is_remote_frame()
    }

    fn id(&self) -> embedded_hal_0_2::can::Id {
        if self.is_standard() {
            let id = unsafe {
                embedded_hal_0_2::can::StandardId::new_unchecked(self.identifier() as u16)
            };
            embedded_hal_0_2::can::Id::Standard(id)
        } else {
            let id = unsafe { embedded_hal_0_2::can::ExtendedId::new_unchecked(self.identifier()) };
            embedded_hal_0_2::can::Id::Extended(id)
        }
    }

    fn dlc(&self) -> usize {
        Frame::dlc(self)
    }

    fn data(&self) -> &[u8] {
        Frame::data(self)
    }
}

impl embedded_can::Frame for Frame {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        let (id, extended) = match id.into() {
            embedded_can::Id::Standard(id) => (id.as_raw() as u32, false),
            embedded_can::Id::Extended(id) => (id.as_raw(), true),
        };

        Self::new(id, extended, data)
    }

    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
        let (id, extended) = match id.into() {
            embedded_can::Id::Standard(id) => (id.as_raw() as u32, false),
            embedded_can::Id::Extended(id) => (id.as_raw(), true),
        };

        Self::new_remote(id, extended, dlc)
    }

    fn is_extended(&self) -> bool {
        Frame::is_extended(self)
    }

    fn is_standard(&self) -> bool {
        !self.is_extended()
    }

    fn is_remote_frame(&self) -> bool {
        Frame::is_remote_frame(self)
    }

    fn is_data_frame(&self) -> bool {
        !self.is_remote_frame()
    }

    fn id(&self) -> embedded_can::Id {
        if self.is_standard() {
            let id = unsafe { embedded_can::StandardId::new_unchecked(self.identifier() as u16) };
            embedded_can::Id::Standard(id)
        } else {
            let id = unsafe { embedded_can::ExtendedId::new_unchecked(self.identifier()) };
            embedded_can::Id::Extended(id)
        }
    }

    fn dlc(&self) -> usize {
        Frame::dlc(self)
    }

    fn data(&self) -> &[u8] {
        Frame::data(self)
    }
}

crate::impl_peripheral!(CAN);
