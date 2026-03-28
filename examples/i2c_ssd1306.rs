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

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

#[cfg(feature = "i2c-legacy")]
fn main() -> anyhow::Result<()> {
    println!("This example uses the new I2C API. Disable the `i2c-legacy` feature.");
    loop {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}

#[cfg(not(feature = "i2c-legacy"))]
fn main() -> anyhow::Result<()> {
    example::main()
}

#[cfg(not(feature = "i2c-legacy"))]
mod example {
    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::i2c::*;
    use esp_idf_hal::peripherals::Peripherals;

    const SSD1306_ADDRESS: u8 = 0x3c;

    pub fn main() -> anyhow::Result<()> {
        esp_idf_hal::sys::link_patches();

        let peripherals = Peripherals::take()?;
        let i2c = peripherals.i2c0;
        let sda = peripherals.pins.gpio5;
        let scl = peripherals.pins.gpio6;

        println!("Starting I2C SSD1306 test");

        let config = I2cMasterBusConfig::new();
        let bus = I2cMasterBus::new(i2c, sda, scl, &config)?;

        let dev_config = I2cMasterDeviceConfig::new().scl_speed_hz(100_000);
        let mut dev = I2cMasterDevice::new(&bus, SSD1306_ADDRESS, &dev_config)?;

        // initialze the display - don't worry about the meaning of these bytes - it's specific to SSD1306
        dev.write(&[0, 0xae])?;
        dev.write(&[0, 0xd4])?;
        dev.write(&[0, 0x80])?;
        dev.write(&[0, 0xa8])?;
        dev.write(&[0, 0x3f])?;
        dev.write(&[0, 0xd3])?;
        dev.write(&[0, 0x00])?;
        dev.write(&[0, 0x40])?;
        dev.write(&[0, 0x8d])?;
        dev.write(&[0, 0x14])?;
        dev.write(&[0, 0xa1])?;
        dev.write(&[0, 0xc8])?;
        dev.write(&[0, 0xda])?;
        dev.write(&[0, 0x12])?;
        dev.write(&[0, 0x81])?;
        dev.write(&[0, 0xcf])?;
        dev.write(&[0, 0xf1])?;
        dev.write(&[0, 0xdb])?;
        dev.write(&[0, 0x40])?;
        dev.write(&[0, 0xa4])?;
        dev.write(&[0, 0xa6])?;
        dev.write(&[0, 0xaf])?;
        dev.write(&[0, 0x20, 0x00])?;

        // fill the display
        for _ in 0..64 {
            let data: [u8; 17] = [
                0x40, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                0xff, 0xff, 0xff,
            ];
            dev.write(&data)?;
        }

        loop {
            // we are sleeping here to make sure the watchdog isn't triggered
            FreeRtos::delay_ms(500);
            dev.write(&[0, 0xa6])?;
            FreeRtos::delay_ms(500);
            dev.write(&[0, 0xa7])?;
        }
    }
}
