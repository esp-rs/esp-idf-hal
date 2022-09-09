//! SPI example with the ST7789 using the ESP-RUST-BOARD 
//!
//! Folowing pins are used:
//! RST       GPIO3
//! DC        GPIO4 
//! BACKLIGHT GPIO5
//! SCLK      GPIO6
//! SDA       GPIO7
//!
//! Depending on your target and the board you are using you have to change the pins.
//!
//! For this example you need to hook up an ST7789 SPI display.
//! The display will display an image on ferris the crab on a black background.

use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use std::thread;
use std::time::Duration;

use embedded_hal::spi::MODE_3;
use embedded_hal_0_2::digital::v2::OutputPin;

use esp_idf_hal::spi;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::units::FromValueType;
use esp_idf_hal::delay::Ets;

use display_interface_spi::SPIInterfaceNoCS;

use embedded_graphics::image::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;

use st7789::{Orientation, ST7789};

fn main() -> anyhow::Result<()> {

    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;

    let rst = peripherals.pins.gpio3.into_output()?;
    let dc = peripherals.pins.gpio4.into_output()?;
    let mut backlight = peripherals.pins.gpio5.into_output()?;
    let sclk = peripherals.pins.gpio6;
    let sda = peripherals.pins.gpio7;
    let sdi = peripherals.pins.gpio8;
    let cs = peripherals.pins.gpio10;

    let mut delay = Ets;

    // configuring the spi interface, note that in order for the ST7789 to work, the data_mode needs to be set to MODE_3
    let config = <spi::config::Config as Default>::default().baudrate(26.MHz().into()).data_mode(MODE_3);
    let spi = spi::Master::<spi::SPI2, _, _, _, _>::new(
        spi,
        spi::Pins {
            sclk,
            sdo: sda,
            sdi: Some(sdi),
            cs: Some(cs),
        },
        config,
    )?;

    // display interface abstraction from SPI and DC
    let di = SPIInterfaceNoCS::new(spi, dc);

    // create driver
    let mut display = ST7789::new(di, rst, 240, 240);

    // initialize
    display.init(&mut delay).unwrap();

    // set default orientation
    display.set_orientation(Orientation::Portrait).unwrap();

    // turn on the backlight
    backlight.set_high()?;
    let raw_image_data = ImageRawLE::new(include_bytes!("../assets/ferris.raw"), 86);
    let ferris = Image::new(&raw_image_data, Point::new(0, 0));

    // draw image on black background
    display.clear(Rgb565::BLACK).unwrap();
    ferris.draw(&mut display).unwrap();

    println!("Image printed!");

    loop {
        thread::sleep(Duration::from_millis(1000));
    }
}
