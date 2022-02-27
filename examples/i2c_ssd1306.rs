//! I2C test with SSD1306
//!
//! Folowing pins are used:
//! SDA     GPIO5
//! SCL     GPIO6
//!
//! Depending on your target and the board you are using you have to change the pins.
//!
//! For this example you need to hook up an SSD1306 I2C display.
//! The display will flash black and white.

use std::thread;
use std::time::Duration;

use embedded_hal::i2c::blocking::I2c;

use esp_idf_hal::i2c;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;

const SSD1306_ADDRESS: u8 = 0x3c;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio5;
    let scl = peripherals.pins.gpio6;

    println!("Starting I2C SSD1306 test");

    let config = <i2c::config::MasterConfig as Default>::default().baudrate(100.kHz().into());
    let mut i2c = i2c::Master::<i2c::I2C0, _, _>::new(i2c, i2c::MasterPins { sda, scl }, config)?;

    // initialze the display - don't worry about the meaning of these bytes - it's specific to SSD1306
    i2c.write(SSD1306_ADDRESS, &[0, 0xae])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xd4])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0x80])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xa8])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0x3f])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xd3])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0x00])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0x40])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0x8d])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0x14])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xa1])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xc8])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xda])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0x12])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0x81])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xcf])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xf1])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xdb])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0x40])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xa4])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xa6])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0xaf])?;
    i2c.write(SSD1306_ADDRESS, &[0, 0x20, 0x00])?;

    // fill the display
    for _ in 0..64 {
        let data: [u8; 17] = [
            0x40, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff,
        ];
        i2c.write(SSD1306_ADDRESS, &data)?;
    }

    loop {
        // we are using thread::sleep here to make sure the watchdog isn't triggered
        thread::sleep(Duration::from_millis(500));
        i2c.write(SSD1306_ADDRESS, &[0, 0xa6])?;
        thread::sleep(Duration::from_millis(500));
        i2c.write(SSD1306_ADDRESS, &[0, 0xa7])?;
    }
}
