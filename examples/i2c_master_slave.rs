//! Test communication between a master and a slave I2C on the same device
//! using the new bus/device and slave APIs.
//!
//! Wiring required, but can be changed in main():
//! - GPIO21 to GPIO18
//! - GPIO22 to GPIO19
//!
//! ESP32-C2/C3 does not have two I2C peripherals, so this example will not work.
//!
//! Description:
//! 1. Master writes 8 bytes, slave receives them
//! 2. Slave loads TX buffer, master reads 8 bytes

#![allow(unused)]
#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

#[cfg(not(esp32))]
fn main() -> anyhow::Result<()> {
    println!("Test only configured for ESP32");
    Ok(())
}

#[cfg(all(esp32, feature = "i2c-legacy"))]
fn main() -> anyhow::Result<()> {
    println!("This example requires the new I2C API. Disable the `i2c-legacy` feature.");

    loop {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}

#[cfg(all(esp32, not(feature = "i2c-legacy")))]
fn main() -> anyhow::Result<()> {
    example::main()
}

#[cfg(all(esp32, not(feature = "i2c-legacy")))]
mod example {
    use std::sync::{Arc, Mutex};

    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::i2c::*;
    use esp_idf_hal::peripherals::Peripherals;

    const SLAVE_ADDR: u8 = 0x22;

    pub fn main() -> anyhow::Result<()> {
        esp_idf_hal::sys::link_patches();

        println!("Starting I2C master/slave test (new API)");

        let peripherals = Peripherals::take()?;

        // Set up the master bus + device on I2C0
        let bus = I2cBusDriver::new(
            peripherals.i2c0,
            peripherals.pins.gpio21,
            peripherals.pins.gpio22,
            &I2cBusConfig::new(),
        )?;
        let mut master = I2cDriver::new(
            &bus,
            SLAVE_ADDR,
            &I2cDeviceConfig::new().scl_speed_hz(100_000),
        )?;

        // Set up the slave on I2C1
        let mut slave = I2cSlaveDriver::new(
            peripherals.i2c1,
            peripherals.pins.gpio18,
            peripherals.pins.gpio19,
            SLAVE_ADDR,
            &I2cSlaveConfig::new().send_buf_depth(256),
        )?;

        // --- Test 1: Master writes, slave receives via callback ---
        println!("-------- TESTING MASTER WRITE -> SLAVE RECEIVE --------");

        let received = Arc::new(Mutex::new(Vec::new()));
        let received_clone = received.clone();

        slave.subscribe(move |data: &[u8]| {
            if let Ok(mut buf) = received_clone.lock() {
                buf.extend_from_slice(data);
            }
        })?;

        let tx_buf: [u8; 8] = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef];
        master.transmit(&tx_buf)?;

        FreeRtos::delay_ms(100);
        let rx_data = received.lock().unwrap().clone();
        println!("Master sent:     {tx_buf:?}");
        println!("Slave received:  {rx_data:?}");

        slave.unsubscribe()?;

        // --- Test 2: Slave transmits, master reads ---
        println!("-------- TESTING SLAVE TRANSMIT -> MASTER READ --------");

        let slave_tx: [u8; 8] = [0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10];
        let mut master_rx: [u8; 8] = [0; 8];

        slave.transmit(&slave_tx)?;
        master.receive(&mut master_rx)?;

        println!("Slave sent:      {slave_tx:?}");
        println!("Master received: {master_rx:?}");

        println!("-------- DONE --------");

        loop {
            FreeRtos::delay_ms(1000);
        }
    }
}
