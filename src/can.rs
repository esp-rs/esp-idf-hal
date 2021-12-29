//! CAN bus peripheral control.
//!
//! It is called Two-Wire Automotive Interface (TWAI) in ESP32 documentation.
//!

use core::marker::PhantomData;

use crate::delay::portMAX_DELAY;
use crate::gpio::*;
use embedded_hal::can::blocking::Can;
use esp_idf_sys::*;

/// CAN timing
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

impl Timing {
    fn timing(&self) -> twai_timing_config_t {
        match self {
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

/// Is used to filter out unwanted CAN IDs (messages).
///
/// Notice that Espressif TWAI (CAN in rest of the world) acceptance filtering
/// works differently than common CAN filtering (for example mask bits are inversed).
/// However here those differences are hidden away from the user and common CAN filtering can be used.
///
/// `mask` is used to determine which bits in the CAN ID are compared with the `filter`.
/// If a mask bit is set to a zero, the corresponding CAN ID bit will be accepted,
/// regardless of the value of the filter bit.
///
/// `filter` determines which bits must be set in the CAN ID to accept the frame,
/// however `mask` can relax those constraints if its bit is set to zero.
///
/// ## Examples
///
/// This shows how 11 bit CAN ID `0x3AA` goes through filtering engine and is finally accepted:
/// ```
/// // can id    [ 0 1 1 1 0 1 0 1 0 1 0 ]
/// // mask      [ 1 0 1 0 0 1 1 1 0 0 0 ]
/// //             1 = compare
/// //             0 = do not care
/// // masked id [ 0 _ 1 _ _ 1 0 1 _ _ _ ]
/// // filter    [ 0 0 1 1 1 1 0 1 0 1 1 ]
///
/// // can id    [ 0 1 1 1 0 1 0 1 0 1 0 ]
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
pub enum Filter {
    // Filter for 11 bit standard CAN IDs
    Standard { filter: u16, mask: u16 },
    // Filter for 29 bit extended CAN IDs
    Extended { filter: u32, mask: u32 },
}

impl Filter {
    /// Filter that allows all standard CAN IDs.
    pub fn standard_allow_all() -> Self {
        Self::Standard {
            filter: 0,
            mask: 0x7FF,
        }
    }

    /// Filter that accepts all extended CAN IDs.
    pub fn extended_allow_all() -> Self {
        Self::Extended {
            filter: 0,
            mask: 0x1FFFFFFF,
        }
    }
}

/// CAN abstraction
pub struct CanBus<TX: OutputPin, RX: InputPin> {
    tx: PhantomData<TX>,
    rx: PhantomData<RX>,
}

impl<TX: OutputPin, RX: InputPin> CanBus<TX, RX> {
    pub fn new(tx: TX, rx: RX, timing: Timing, filter: Filter) -> Result<Self, EspError> {
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

        let timing_config = timing.timing();

        // modify filter and mask to be compatible with TWAI acceptance filter
        let (filter, mask) = match filter {
            Filter::Standard { filter, mask } => ((filter as u32) << 21, !((mask as u32) << 21)),
            Filter::Extended { filter, mask } => (filter << 3, !(mask << 3)),
        };

        let filter_config = twai_filter_config_t {
            acceptance_code: filter,
            acceptance_mask: mask,
            single_filter: true,
        };

        esp!(unsafe { twai_driver_install(&general_config, &timing_config, &filter_config) })?;
        esp!(unsafe { twai_start() })?;

        Ok(Self {
            tx: PhantomData,
            rx: PhantomData,
        })
    }
}

pub struct Frame(twai_message_t);

impl embedded_hal::can::Frame for Frame {
    fn new(id: impl Into<embedded_hal::can::Id>, data: &[u8]) -> Option<Self> {
        let dlc = data.len();

        if dlc <= 8 {
            // unions are not very well supported in rust
            // therefore setting those union flags is quite hairy
            let mut flags = twai_message_t__bindgen_ty_1::default();

            let id: embedded_hal::can::Id = id.into();
            let id = match id {
                embedded_hal::can::Id::Standard(id) => id.as_raw() as u32,
                embedded_hal::can::Id::Extended(id) => {
                    // set bits in an union
                    unsafe { flags.__bindgen_anon_1.set_extd(1) };
                    unsafe { flags.__bindgen_anon_1.set_ss(1) };
                    id.as_raw()
                }
            };

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

    fn new_remote(id: impl Into<embedded_hal::can::Id>, dlc: usize) -> Option<Self> {
        if dlc <= 8 {
            // unions are not very well supported in rust
            // therefore setting those union flags is quite hairy
            let mut flags = twai_message_t__bindgen_ty_1::default();

            let id: embedded_hal::can::Id = id.into();
            let id = match id {
                embedded_hal::can::Id::Standard(id) => id.as_raw() as u32,
                embedded_hal::can::Id::Extended(id) => {
                    // set bits in an union
                    unsafe { flags.__bindgen_anon_1.set_rtr(1) };
                    unsafe { flags.__bindgen_anon_1.set_ss(1) };
                    id.as_raw()
                }
            };

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

    fn is_extended(&self) -> bool {
        unsafe { self.0.__bindgen_anon_1.__bindgen_anon_1.extd() == 1 }
    }

    fn is_standard(&self) -> bool {
        !self.is_extended()
    }

    fn is_remote_frame(&self) -> bool {
        unsafe { self.0.__bindgen_anon_1.__bindgen_anon_1.rtr() == 1 }
    }

    fn is_data_frame(&self) -> bool {
        !self.is_remote_frame()
    }

    fn id(&self) -> embedded_hal::can::Id {
        if unsafe { self.0.__bindgen_anon_1.__bindgen_anon_1.extd() == 1 } {
            let id =
                unsafe { embedded_hal::can::StandardId::new_unchecked(self.0.identifier as u16) };
            embedded_hal::can::Id::Standard(id)
        } else {
            let id = unsafe { embedded_hal::can::ExtendedId::new_unchecked(self.0.identifier) };
            embedded_hal::can::Id::Extended(id)
        }
    }

    fn dlc(&self) -> usize {
        self.0.data_length_code as usize
    }

    fn data(&self) -> &[u8] {
        &self.0.data
    }
}
