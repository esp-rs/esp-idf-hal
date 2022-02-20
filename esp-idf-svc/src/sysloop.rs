use esp_idf_sys::*;

use crate::eventloop::EspSystemEventLoop;

#[derive(Debug)]
pub struct EspSysLoopStack(EspSystemEventLoop);

impl EspSysLoopStack {
    pub fn new() -> Result<Self, EspError> {
        Ok(EspSysLoopStack(EspSystemEventLoop::new()?))
    }

    pub fn get_loop(&self) -> &EspSystemEventLoop {
        &self.0
    }
}
