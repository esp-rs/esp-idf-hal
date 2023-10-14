use esp_idf_hal::peripherals::*;
use esp_idf_hal::sys::EspError;
use esp_idf_hal::task::*;
use esp_idf_hal::timer::*;

fn main() -> Result<(), EspError> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_hal::sys::link_patches();

    let per = Peripherals::take()?;

    let mut timer = TimerDriver::new(per.timer00, &TimerConfig::new())?;

    block_on(async {
        loop {
            timer.delay(timer.tick_hz()).await?; // Every second

            println!("Tick");
        }
    })
}
