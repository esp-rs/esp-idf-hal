use crate::log::Logger;

static LOGGER: Logger = Logger;

#[cfg(feature = "binstart")]
extern "C" {
    fn main(p1: isize, p2: *const *const u8) -> isize;
}

#[cfg(feature = "libstart")]
extern "Rust" {
    fn main() -> anyhow::Result<()>;
}

#[no_mangle]
pub extern "C" fn app_main() {
    log::set_logger(&LOGGER).map(|()| LOGGER.initialize()).unwrap();

    #[cfg(feature = "binstart")]
    match unsafe {main(0, core::ptr::null())} {
        0 => log::error!("Unexpected program exit!\n(no error reported)"),
        n => log::error!("Unexpected program exit!\n{}", n)
    }

    #[cfg(feature = "libstart")]
    match unsafe {main()} {
        Ok(()) => log::error!("Unexpected program exit!\n(no error reported)"),
        Err(err) => log::error!("Unexpected program exit!\n{:?}", err)
    }

    log::warn!("Will restart now...");
    panic!();
}
