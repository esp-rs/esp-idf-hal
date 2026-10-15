//! Checks that an ESP32-S3 USB pad can be reclaimed as a GPIO output.
//!
//! Connect GPIO19 to GPIO4 with a jumper before running this example. Use a UART
//! console, since reclaiming GPIO19 disables USB Serial/JTAG on GPIO19/20.
//! Run after a reset, before another driver has configured either USB pad.
//! Build and flash with:
//! `MCU=esp32s3 cargo espflash flash --target xtensa-esp32s3-espidf --bin gpio_usb_pad --monitor`
//!
//! GPIO4 samples the physical output rather than the GPIO19 output latch: the
//! latch can report high while USB still owns the pad. Without the fix, the
//! first high-level check fails on an ESP32-S3 with USB pad ownership enabled.

#[cfg(esp32s3)]
fn main() -> anyhow::Result<()> {
    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::gpio::{PinDriver, Pull};
    use esp_idf_hal::peripherals::Peripherals;

    esp_idf_hal::sys::link_patches();

    let peripherals = Peripherals::take()?;
    let input = PinDriver::input(peripherals.pins.gpio4, Pull::Down)?;
    let mut output = PinDriver::output(peripherals.pins.gpio19)?;

    for _ in 0..4 {
        output.set_high()?;
        FreeRtos::delay_ms(10);
        let high = input.is_high();

        output.set_low()?;
        FreeRtos::delay_ms(10);
        let low = input.is_low();

        anyhow::ensure!(
            high,
            "GPIO19 did not drive high; check the GPIO19-GPIO4 jumper"
        );
        anyhow::ensure!(
            low,
            "GPIO19 did not drive low; check the GPIO19-GPIO4 jumper"
        );
    }

    println!("GPIO19 USB-pad output loopback passed");
    Ok(())
}

#[cfg(not(esp32s3))]
fn main() -> anyhow::Result<()> {
    println!("This GPIO USB-pad loopback example requires an ESP32-S3");
    Ok(())
}
