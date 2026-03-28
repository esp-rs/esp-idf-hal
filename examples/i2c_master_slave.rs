//! Test communication between a master and a slave I2C on the same device
//! using the new master bus/device and slave APIs.
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
    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::i2c::*;
    use esp_idf_hal::peripherals::Peripherals;

    const SLAVE_ADDR: u8 = 0x22;

    pub fn main() -> anyhow::Result<()> {
        esp_idf_hal::sys::link_patches();

        println!("Starting I2C master/slave test (new API)");

        let peripherals = Peripherals::take()?;

        // Set up the master bus + device on I2C0
        let bus = I2cMasterBus::new(
            peripherals.i2c0,
            peripherals.pins.gpio21,
            peripherals.pins.gpio22,
            &I2cMasterBusConfig::new(),
        )?;
        let mut master = I2cMasterDevice::new(
            &bus,
            SLAVE_ADDR,
            &I2cMasterDeviceConfig::new().scl_speed_hz(100_000),
        )?;

        // Set up the slave on I2C1
        let mut slave = I2cSlaveDevice::new(
            peripherals.i2c1,
            peripherals.pins.gpio18,
            peripherals.pins.gpio19,
            SLAVE_ADDR,
            &I2cSlaveDeviceConfig::new().send_buf_depth(256),
        )?;

        // --- Test 1: Master writes, slave receives ---
        println!("-------- TESTING MASTER WRITE -> SLAVE RECEIVE --------");

        let tx_buf: [u8; 8] = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef];
        let mut rx_buf: [u8; 8] = [0; 8];

        // Initiate non-blocking receive on slave first
        slave.receive(&mut rx_buf)?;

        // Master sends data
        master.write(&tx_buf)?;

        // Give the transfer time to complete
        FreeRtos::delay_ms(100);
        println!("Master sent:     {tx_buf:?}");
        println!("Slave received:  {rx_buf:?}");

        // --- Test 2: Slave transmits, master reads ---
        println!("-------- TESTING SLAVE TRANSMIT -> MASTER READ --------");

        let slave_tx: [u8; 8] = [0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10];
        let mut master_rx: [u8; 8] = [0; 8];

        // Load data into slave TX buffer
        slave.transmit(&slave_tx)?;

        // Master reads
        master.read(&mut master_rx)?;

        println!("Slave sent:      {slave_tx:?}");
        println!("Master received: {master_rx:?}");

        println!("-------- DONE --------");

        loop {
            FreeRtos::delay_ms(1000);
        }
    }
}
