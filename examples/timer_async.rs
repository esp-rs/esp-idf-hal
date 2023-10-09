use esp_idf_hal::sys::EspError;

fn main() -> Result<(), EspError> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_hal::sys::link_patches();

    let per = esp_idf_hal::peripherals::Peripherals::take().unwrap();

    let timer_conf = esp_idf_hal::timer::config::Config::new().auto_reload(true);
    let mut timer = esp_idf_hal::timer::TimerDriver::new(per.timer00, &timer_conf)?;

    esp_idf_hal::task::block_on(async move {
        loop {
            timer.delay(timer.tick_hz()).await?; // Every second
            println!("Tick");
        }
    })
}
