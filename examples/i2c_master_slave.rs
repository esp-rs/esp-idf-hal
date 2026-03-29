//! Simple I2C master/slave loopback example.
//!
//! Demonstrates the new bus/device master API and the v2 slave driver
//! on a single ESP32 with two I2C peripherals cross-wired.
//!
//! Requires ESP-IDF v6.0+ or v5.x with `CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2=y`.
//!
//! Wiring:
//! - GPIO21 ↔ GPIO18  (SDA)
//! - GPIO22 ↔ GPIO19  (SCL)
//!
//! Flash and monitor:
//!   cargo espflash flash --example i2c_master_slave --monitor

#![allow(unexpected_cfgs)]

#[cfg(not(esp32))]
fn main() -> anyhow::Result<()> {
    println!("Example only configured for ESP32");
    Ok(())
}

#[cfg(all(esp32, feature = "i2c-legacy"))]
fn main() -> anyhow::Result<()> {
    println!("This example requires the new I2C API. Disable the `i2c-legacy` feature.");
    loop {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}

#[cfg(all(
    esp32,
    not(feature = "i2c-legacy"),
    not(any(
        esp_idf_i2c_enable_slave_driver_version_2,
        esp_idf_version_at_least_6_0_0
    ))
))]
fn main() -> anyhow::Result<()> {
    println!("This example requires the v2 slave driver.");
    println!("Use ESP-IDF v6.0+ or set CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2=y");
    loop {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}

#[cfg(all(
    esp32,
    not(feature = "i2c-legacy"),
    any(
        esp_idf_i2c_enable_slave_driver_version_2,
        esp_idf_version_at_least_6_0_0
    )
))]
fn main() -> anyhow::Result<()> {
    example::main()
}

#[cfg(all(
    esp32,
    not(feature = "i2c-legacy"),
    any(
        esp_idf_i2c_enable_slave_driver_version_2,
        esp_idf_version_at_least_6_0_0
    )
))]
mod example {
    use std::sync::{Arc, Mutex};

    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::i2c::*;
    use esp_idf_hal::peripherals::Peripherals;

    const SLAVE_ADDR: u8 = 0x22;

    pub fn main() -> anyhow::Result<()> {
        esp_idf_hal::sys::link_patches();

        println!("I2C master/slave loopback example");

        let peripherals = Peripherals::take()?;

        // Master on I2C0
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

        // Slave on I2C1
        let mut slave = I2cSlaveDriver::new(
            peripherals.i2c1,
            peripherals.pins.gpio18,
            peripherals.pins.gpio19,
            SLAVE_ADDR,
            &I2cSlaveConfig::new().send_buf_depth(256),
        )?;

        // -- Master writes, slave receives via subscribe callback --
        println!("\n--- Master write -> Slave receive ---");

        let received = Arc::new(Mutex::new(Vec::new()));
        let clone = received.clone();
        slave.subscribe(move |data: &[u8]| {
            if let Ok(mut buf) = clone.lock() {
                buf.extend_from_slice(data);
            }
        })?;

        let tx_buf = [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF];
        master.transmit(&tx_buf)?;

        let rx_data = received.lock().unwrap().clone();
        println!("Master sent:     {tx_buf:02x?}");
        println!("Slave received:  {rx_data:02x?}");

        slave.unsubscribe()?;

        // -- Slave transmits, master reads --
        println!("\n--- Slave transmit -> Master read ---");

        let slave_tx = [0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10];
        let mut master_rx = [0u8; 8];

        slave.transmit(&slave_tx)?;
        master.receive(&mut master_rx)?;

        println!("Slave sent:      {slave_tx:02x?}");
        println!("Master received: {master_rx:02x?}");

        // -- Write+Read (repeated START) --
        println!("\n--- Master write_read (repeated START) ---");

        let reg_addr = [0x42];
        let response = [0xCA, 0xFE];
        slave.transmit(&response)?;

        let mut read_back = [0u8; 2];
        master.transmit_receive(&reg_addr, &mut read_back)?;

        println!("Register addr:   {reg_addr:02x?}");
        println!("Read back:       {read_back:02x?}");

        println!("\nDone.");

        loop {
            FreeRtos::delay_ms(1000);
        }
    }
}
