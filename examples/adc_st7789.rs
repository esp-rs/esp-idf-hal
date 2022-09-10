//! ADC example reading a value from a pin and writing it to the ST7789 screen
//!
//! Folowing pins are used:
//! ADC_PIN   GPIO0
//! RST       GPIO3
//! DC        GPIO4 
//! BACKLIGHT GPIO5
//! SCLK      GPIO6
//! SDA       GPIO7
//!
//! Depending on your target and the board you are using you have to change the pins.
//!
//! For this example you need to hook up an ST7789 SPI display and an analog input for example a potentiometer
//!

use esp_idf_sys::{self as _}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use std::thread;
use std::time::Duration;
use core::fmt::Write;
use embedded_hal::spi::MODE_3;
use embedded_hal_0_2::digital::v2::OutputPin;
use embedded_hal_0_2::adc::OneShot;

use arrayvec::ArrayString;

use esp_idf_hal::spi;
use esp_idf_hal::units::FromValueType;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::adc::*;
use esp_idf_hal::adc::config::Config;

use display_interface_spi::SPIInterfaceNoCS;

use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, iso_8859_5::FONT_9X18},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Baseline, Text},
    primitives::rectangle::Rectangle,
};

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

    // initializing adc
    let mut adc1 = PoweredAdc::new(
        peripherals.adc1,
        Config::new().calibration(true),
    )?;

    // configuring pin0 to analog read, you can regulate the adc input voltage range depending on your need
    // for this example we use the attenuation of 11db which sets the input voltage range to around 0-3.6V
    let mut adc_pin = peripherals.pins.gpio0.into_analog_atten_11db()?;

    // set text style and color
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X18)
        .text_color(Rgb565::WHITE)
        .build();

    // defining area to be refreshed to avoid refreshing the entire screen
    let area = Rectangle::new(Point::zero(), Size::new(40, 20));

    // clears the display
    display.clear(Rgb565::BLACK).unwrap();

    loop {

        // you can change the sleep duration depending on how often you want to sample
        thread::sleep(Duration::from_millis(200));

        //clears the specified area on the screen 
        display.fill_solid(&area, Rgb565::BLACK).unwrap();

        // creating a buffer to hold our value
        let mut buf = ArrayString::<8>::new();
        
        // reading the adc value
        let mut adc_value: u16 = adc1.read(&mut adc_pin).unwrap();

        // writing the adc_value to the buffer
        write!(&mut buf, "{}", &mut adc_value).expect("Failed to write to buffer");

        // writing value from the buffer on the screen
        Text::with_baseline(&buf, Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
    }
}
