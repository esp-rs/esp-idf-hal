use std::env;

use anyhow::Result;
use log::{error, warn};

use crate::log::Logger;

static LOGGER: Logger = Logger;

extern "Rust" {
    fn main() -> Result<()>;
}

#[no_mangle]
pub extern "C" fn app_main() {
    log::set_logger(&LOGGER).map(|()| LOGGER.initialize()).unwrap();

    match unsafe {main()} {
        Ok(()) => error!("Unexpected program exit!\n(no error reported)"),
        Err(err) => error!("Unexpected program exit!\n{:?}", err)
    }

    warn!("Will restart now...");
    panic!();
}
