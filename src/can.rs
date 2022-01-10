//! CAN bus peripheral control.
//!
//! It is called Two-Wire Automotive Interface (TWAI) in ESP32 documentation.
//!
//! # Example
//!
//! Create a CAN peripheral and then transmit and receive a message.
//! ```
//! use embedded_hal::can::nb::Can;
//! use embedded_hal::can::Frame;
//! use embedded_hal::can::StandardId;
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
//! let mut can = can::CanBus::new(peripherals.can, pins.gpio5, pins.gpio4, config).unwrap();
//!
//! let tx_frame = can::Frame::new(StandardId::new(0x042).unwrap(), &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
//! nb::block!(can.transmit(&tx_frame)).unwrap();
//!
//! if let Ok(rx_frame) = nb::block!(can.receive()) {
//!    info!("rx {:}:", rx_frame);
//! }
//! ```

use core::marker::PhantomData;

use esp_idf_sys::*;

use crate::delay::portMAX_DELAY;
use crate::gpio::*;

crate::embedded_hal_error!(
    CanError,
    embedded_hal::can::Error,
    embedded_hal::can::ErrorKind
);

pub mod config {
    use esp_idf_sys::*;

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
    }

    impl From<Timing> for twai_timing_config_t {
        fn from(resolution: Timing) -> Self {
            match resolution {
                Timing::B25K => twai_timing_config_t {
                    brp: 128,
                    tseg_1: 16,
                    tseg_2: 8,
                    sjw: 3,
                    triple_sampling: false,
                },
                Timing::B50K => twai_timing_config_t {
                    brp: 80,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                },
                Timing::B100K => twai_timing_config_t {
                    brp: 40,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                },
                Timing::B125K => twai_timing_config_t {
                    brp: 32,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                },
                Timing::B250K => twai_timing_config_t {
                    brp: 16,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                },
                Timing::B500K => twai_timing_config_t {
                    brp: 8,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
                },
                Timing::B800K => twai_timing_config_t {
                    brp: 4,
                    tseg_1: 16,
                    tseg_2: 8,
                    sjw: 3,
                    triple_sampling: false,
                },
                Timing::B1M => twai_timing_config_t {
                    brp: 4,
                    tseg_1: 15,
                    tseg_2: 4,
                    sjw: 3,
                    triple_sampling: false,
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
    pub struct Config {
        pub timing: Timing,
        pub filter: Filter,
    }

    impl Config {
        pub fn new() -> Self {
            Default::default()
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
    }
}

/// CAN abstraction
pub struct CanBus<TX: OutputPin, RX: InputPin> {
    can: CAN,
    tx: TX,
    rx: RX,
}

unsafe impl<TX: OutputPin, RX: InputPin> Send for CanBus<TX, RX> {}

impl<TX: OutputPin, RX: InputPin> CanBus<TX, RX> {
    pub fn new(can: CAN, tx: TX, rx: RX, config: config::Config) -> Result<Self, EspError> {
        let general_config = twai_general_config_t {
            mode: twai_mode_t_TWAI_MODE_NORMAL,
            tx_io: tx.pin(),
            rx_io: rx.pin(),
            clkout_io: -1,
            bus_off_io: -1,
            tx_queue_len: 5,
            rx_queue_len: 5,
            alerts_enabled: TWAI_ALERT_NONE,
            clkout_divider: 0,
            intr_flags: ESP_INTR_FLAG_LEVEL1 as i32,
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

        Ok(Self { can, tx, rx })
    }

    pub fn release(self) -> Result<(CAN, TX, RX), EspError> {
        esp!(unsafe { twai_stop() })?;
        esp!(unsafe { twai_driver_uninstall() })?;

        Ok((self.can, self.tx, self.rx))
    }

    fn transmit_internal(&mut self, frame: &Frame, delay: TickType_t) -> Result<(), EspError> {
        esp!(unsafe { twai_transmit(&frame.0, delay) })
    }

    fn receive_internal(&mut self, delay: TickType_t) -> Result<Frame, EspError> {
        let mut rx_msg = twai_message_t {
            ..Default::default()
        };

        match esp_result!(unsafe { twai_receive(&mut rx_msg, delay) }, ()) {
            Ok(_) => Ok(Frame(rx_msg)),
            Err(err) => Err(err),
        }
    }
}

// Requires embedded-hal V0.2.7 which is not released yet
// impl<TX: OutputPin, RX: InputPin> embedded_hal_0_2::blocking::can::Can for CanBus<TX, RX> {
//     type Frame = Frame;
//     type Error = CanError;

//     fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
//         self.transmit_internal(frame, portMAX_DELAY)
//             .map_err(CanError::other)
//     }

//     fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
//         self.receive_internal(portMAX_DELAY)
//             .map_err(CanError::other)
//     }
// }

impl<TX: OutputPin, RX: InputPin> embedded_hal::can::blocking::Can for CanBus<TX, RX> {
    type Frame = Frame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        self.transmit_internal(frame, portMAX_DELAY)
            .map_err(CanError::other)
    }

    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        self.receive_internal(portMAX_DELAY)
            .map_err(CanError::other)
    }
}

// Requires embedded-hal V0.2.7 which is not released yet
// impl<TX: OutputPin, RX: InputPin> embedded_hal_0_2::can::nb::Can for CanBus<TX, RX> {
//     type Frame = Frame;
//     type Error = CanError;

//     fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
//         match self.transmit_internal(frame, 0) {
//             Ok(_) => Ok(None),
//             Err(e) if e.code() == ESP_FAIL => Err(nb::Error::WouldBlock),
//             Err(e) if e.code() == ESP_ERR_TIMEOUT as i32 => Err(nb::Error::WouldBlock),
//             Err(e) => Err(nb::Error::Other(CanError::other(e))),
//         }
//     }

//     fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
//         match self.receive_internal(0) {
//             Ok(frame) => Ok(frame),
//             Err(e) if e.code() == ESP_ERR_TIMEOUT as i32 => Err(nb::Error::WouldBlock),
//             Err(e) => Err(nb::Error::Other(CanError::other(e))),
//         }
//     }
// }

impl<TX: OutputPin, RX: InputPin> embedded_hal::can::nb::Can for CanBus<TX, RX> {
    type Frame = Frame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        match self.transmit_internal(frame, 0) {
            Ok(_) => Ok(None),
            Err(e) if e.code() == ESP_FAIL => Err(nb::Error::WouldBlock),
            Err(e) if e.code() == ESP_ERR_TIMEOUT as i32 => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(CanError::other(e))),
        }
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        match self.receive_internal(0) {
            Ok(frame) => Ok(frame),
            Err(e) if e.code() == ESP_ERR_TIMEOUT as i32 => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(CanError::other(e))),
        }
    }
}

pub struct Frame(twai_message_t);

impl Frame {
    fn new(id: u32, extended: bool, data: &[u8]) -> Option<Self> {
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

    fn new_remote(id: u32, extended: bool, dlc: usize) -> Option<Self> {
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

    fn get_extended(&self) -> bool {
        unsafe { self.0.__bindgen_anon_1.__bindgen_anon_1.extd() == 1 }
    }

    fn get_remote_frame(&self) -> bool {
        unsafe { self.0.__bindgen_anon_1.__bindgen_anon_1.rtr() == 1 }
    }

    fn get_identifier(&self) -> u32 {
        self.0.identifier
    }

    fn get_dlc(&self) -> usize {
        self.0.data_length_code as usize
    }

    fn get_data(&self) -> &[u8] {
        &self.0.data[..self.get_dlc()]
    }
}

impl core::fmt::Display for Frame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "Frame {{ id: {}, remote: {}, data: {:?} }}",
            self.get_identifier(),
            self.get_remote_frame(),
            self.get_data()
        )
    }
}

// Requires embedded-hal V0.2.7 which is not released yet
// impl embedded_hal_0_2::can::Frame for Frame {
//     fn new(id: impl Into<embedded_hal_0_2::can::Id>, data: &[u8]) -> Option<Self> {
//         let (id, extended) = match id.into() {
//             embedded_hal::can::Id::Standard(id) => (id.as_raw() as u32, false),
//             embedded_hal::can::Id::Extended(id) => (id.as_raw(), true),
//         };

//         Self::new(id, extended, data)
//     }

//     fn new_remote(id: impl Into<embedded_hal_0_2::can::Id>, dlc: usize) -> Option<Self> {
//         let (id, extended) = match id.into() {
//             embedded_hal::can::Id::Standard(id) => (id.as_raw() as u32, false),
//             embedded_hal::can::Id::Extended(id) => (id.as_raw(), true),
//         };

//         Self::new_remote(id, extended, dlc)
//     }

//     fn is_extended(&self) -> bool {
//         self.get_extended()
//     }

//     fn is_standard(&self) -> bool {
//         !self.is_extended()
//     }

//     fn is_remote_frame(&self) -> bool {
//         self.get_remote_frame()
//     }

//     fn is_data_frame(&self) -> bool {
//         !self.is_remote_frame()
//     }

//     fn id(&self) -> embedded_hal_0_2::can::Id {
//         if self.is_standard() {
//             let id = unsafe {
//                 embedded_hal_0_2::can::StandardId::new_unchecked(self.get_identifier() as u16)
//             };
//             embedded_hal_0_2::can::Id::Standard(id)
//         } else {
//             let id =
//                 unsafe { embedded_hal_0_2::can::ExtendedId::new_unchecked(self.get_identifier()) };
//             embedded_hal_0_2::can::Id::Extended(id)
//         }
//     }

//     fn dlc(&self) -> usize {
//         self.get_dlc()
//     }

//     fn data(&self) -> &[u8] {
//         self.get_data()
//     }
// }

impl embedded_hal::can::Frame for Frame {
    fn new(id: impl Into<embedded_hal::can::Id>, data: &[u8]) -> Option<Self> {
        let (id, extended) = match id.into() {
            embedded_hal::can::Id::Standard(id) => (id.as_raw() as u32, false),
            embedded_hal::can::Id::Extended(id) => (id.as_raw(), true),
        };

        Self::new(id, extended, data)
    }

    fn new_remote(id: impl Into<embedded_hal::can::Id>, dlc: usize) -> Option<Self> {
        let (id, extended) = match id.into() {
            embedded_hal::can::Id::Standard(id) => (id.as_raw() as u32, false),
            embedded_hal::can::Id::Extended(id) => (id.as_raw(), true),
        };

        Self::new_remote(id, extended, dlc)
    }

    fn is_extended(&self) -> bool {
        self.get_extended()
    }

    fn is_standard(&self) -> bool {
        !self.is_extended()
    }

    fn is_remote_frame(&self) -> bool {
        self.get_remote_frame()
    }

    fn is_data_frame(&self) -> bool {
        !self.is_remote_frame()
    }

    fn id(&self) -> embedded_hal::can::Id {
        if self.is_standard() {
            let id = unsafe {
                embedded_hal::can::StandardId::new_unchecked(self.get_identifier() as u16)
            };
            embedded_hal::can::Id::Standard(id)
        } else {
            let id = unsafe { embedded_hal::can::ExtendedId::new_unchecked(self.get_identifier()) };
            embedded_hal::can::Id::Extended(id)
        }
    }

    fn dlc(&self) -> usize {
        self.get_dlc()
    }

    fn data(&self) -> &[u8] {
        self.get_data()
    }
}

pub struct CAN(PhantomData<*const ()>);

impl CAN {
    /// # Safety
    ///
    /// Care should be taken not to instnatiate this CAN instance, if it is already instantiated and used elsewhere
    pub unsafe fn new() -> Self {
        CAN(PhantomData)
    }
}

unsafe impl Send for CAN {}
