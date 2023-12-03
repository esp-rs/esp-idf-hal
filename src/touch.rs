use core::borrow::{Borrow, BorrowMut};

use esp_idf_sys::{
    esp, touch_fsm_mode_t, touch_fsm_mode_t_TOUCH_FSM_MODE_MAX, touch_fsm_mode_t_TOUCH_FSM_MODE_SW,
    touch_fsm_mode_t_TOUCH_FSM_MODE_TIMER, touch_pad_config, touch_pad_fsm_start,
    touch_pad_fsm_stop, touch_pad_init, touch_pad_read_raw_data, touch_pad_set_fsm_mode,
    touch_pad_t, touch_pad_t_TOUCH_PAD_NUM0, touch_pad_t_TOUCH_PAD_NUM1,
    touch_pad_t_TOUCH_PAD_NUM10, touch_pad_t_TOUCH_PAD_NUM11, touch_pad_t_TOUCH_PAD_NUM12,
    touch_pad_t_TOUCH_PAD_NUM13, touch_pad_t_TOUCH_PAD_NUM14, touch_pad_t_TOUCH_PAD_NUM2,
    touch_pad_t_TOUCH_PAD_NUM3, touch_pad_t_TOUCH_PAD_NUM4, touch_pad_t_TOUCH_PAD_NUM5,
    touch_pad_t_TOUCH_PAD_NUM6, touch_pad_t_TOUCH_PAD_NUM7, touch_pad_t_TOUCH_PAD_NUM8,
    touch_pad_t_TOUCH_PAD_NUM9, EspError,
};

#[cfg(any(esp32, esp32s2, esp32s3))]
#[derive(Copy, Clone)]
pub enum FsmMode {
    Timer,
    SW,
    Max,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl From<FsmMode> for touch_fsm_mode_t {
    fn from(value: FsmMode) -> Self {
        match value {
            FsmMode::Timer => touch_fsm_mode_t_TOUCH_FSM_MODE_TIMER,
            FsmMode::SW => touch_fsm_mode_t_TOUCH_FSM_MODE_SW,
            FsmMode::Max => touch_fsm_mode_t_TOUCH_FSM_MODE_MAX,
        }
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
#[derive(Copy, Clone)]
pub enum TouchPad {
    Pad0,
    Pad1,
    Pad2,
    Pad3,
    Pad4,
    Pad5,
    Pad6,
    Pad7,
    Pad8,
    Pad9,
    Pad10,
    Pad11,
    Pad12,
    Pad13,
    Pad14,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl From<TouchPad> for touch_pad_t {
    fn from(value: TouchPad) -> Self {
        match value {
            TouchPad::Pad0 => touch_pad_t_TOUCH_PAD_NUM0,
            TouchPad::Pad1 => touch_pad_t_TOUCH_PAD_NUM1,
            TouchPad::Pad2 => touch_pad_t_TOUCH_PAD_NUM2,
            TouchPad::Pad3 => touch_pad_t_TOUCH_PAD_NUM3,
            TouchPad::Pad4 => touch_pad_t_TOUCH_PAD_NUM4,
            TouchPad::Pad5 => touch_pad_t_TOUCH_PAD_NUM5,
            TouchPad::Pad6 => touch_pad_t_TOUCH_PAD_NUM6,
            TouchPad::Pad7 => touch_pad_t_TOUCH_PAD_NUM7,
            TouchPad::Pad8 => touch_pad_t_TOUCH_PAD_NUM8,
            TouchPad::Pad9 => touch_pad_t_TOUCH_PAD_NUM9,
            TouchPad::Pad10 => touch_pad_t_TOUCH_PAD_NUM10,
            TouchPad::Pad11 => touch_pad_t_TOUCH_PAD_NUM11,
            TouchPad::Pad12 => touch_pad_t_TOUCH_PAD_NUM12,
            TouchPad::Pad13 => touch_pad_t_TOUCH_PAD_NUM13,
            TouchPad::Pad14 => touch_pad_t_TOUCH_PAD_NUM14,
        }
    }
}

pub struct TouchConfig {
    fsm_mode: FsmMode,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
pub struct TouchDriver {
    config: TouchConfig,
}

pub struct TouchPadDriver<T>
where
    T: BorrowMut<TouchDriver>,
{
    _touch: T,
    pad: TouchPad,
}

impl TouchPadDriver<TouchDriver> {
    pub fn new_single_started(pad: TouchPad, config: TouchConfig) -> Result<Self, EspError> {
        let mut touch = TouchDriver::new(config)?;
        esp!(unsafe { touch_pad_config(pad.into()) })?;
        touch.start()?;
        Ok(Self { _touch: touch, pad })
    }
}

impl<T> TouchPadDriver<T>
where
    T: BorrowMut<TouchDriver>,
{
    pub fn read_raw_data(&self) -> Result<u32, EspError> {
        let mut raw: u32 = 0;
        let result = esp!(unsafe { touch_pad_read_raw_data(self.borrow().pad.into(), &mut raw) });
        result.map(|_| raw)
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl TouchDriver {
    pub fn new(config: TouchConfig) -> Result<Self, EspError> {
        unsafe {
            esp!(touch_pad_init())?;
        }

        Ok(TouchDriver { config })
    }

    pub fn start(&mut self) -> Result<(), EspError> {
        unsafe {
            esp!(touch_pad_set_fsm_mode(self.config.fsm_mode.into()))?;
            esp!(touch_pad_fsm_start())?;
        }

        Ok(())
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl Drop for TouchDriver {
    fn drop(&mut self) {
        esp!(unsafe { touch_pad_fsm_stop() }).unwrap()
    }
}
