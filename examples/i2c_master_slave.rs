//! Hardware validation for the I2C master/slave driver.
//!
//! Runs on a single ESP32 using both I2C peripherals in loopback:
//! I2C0 as master, I2C1 as slave, with SDA/SCL cross-wired.
//!
//! Wiring (configurable in `main()`):
//! - GPIO21 ↔ GPIO18  (SDA)
//! - GPIO22 ↔ GPIO19  (SCL)
//!
//! Requires a chip with two I2C peripherals (not ESP32-C2/C3).
//!
//! Flash and monitor:
//!   cargo espflash flash --example i2c_master_slave --monitor

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
    tests::main()
}

#[cfg(all(esp32, not(feature = "i2c-legacy")))]
mod tests {
    use std::sync::{Arc, Mutex};

    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::i2c::*;
    use esp_idf_hal::peripherals::Peripherals;
    use esp_idf_hal::sys::ESP_ERR_NOT_SUPPORTED;

    const SLAVE_ADDR: u8 = 0x22;
    const UNUSED_ADDR: u8 = 0x7E;

    // ---------------------------------------------------------------
    // Test runner
    // ---------------------------------------------------------------

    struct TestRunner {
        passed: u32,
        failed: u32,
    }

    impl TestRunner {
        fn new() -> Self {
            Self {
                passed: 0,
                failed: 0,
            }
        }

        fn pass(&mut self, name: &str) {
            self.passed += 1;
            println!("  PASS: {name}");
        }

        fn fail(&mut self, name: &str, detail: &str) {
            self.failed += 1;
            println!("  FAIL: {name} — {detail}");
        }

        fn assert_eq(&mut self, name: &str, expected: &[u8], actual: &[u8]) {
            if expected == actual {
                self.pass(name);
            } else {
                self.fail(
                    name,
                    &format!("expected {expected:02x?}, got {actual:02x?}"),
                );
            }
        }

        fn assert_ok<T>(&mut self, name: &str, result: &Result<T, impl core::fmt::Debug>) {
            match result {
                Ok(_) => self.pass(name),
                Err(e) => self.fail(name, &format!("{e:?}")),
            }
        }

        fn assert_err<T: core::fmt::Debug>(
            &mut self,
            name: &str,
            result: &Result<T, esp_idf_hal::sys::EspError>,
        ) {
            match result {
                Err(_) => self.pass(name),
                Ok(v) => self.fail(name, &format!("expected error, got {v:?}")),
            }
        }

        fn assert_err_code<T: core::fmt::Debug>(
            &mut self,
            name: &str,
            result: &Result<T, esp_idf_hal::sys::EspError>,
            expected_code: i32,
        ) {
            match result {
                Err(e) if e.code() == expected_code => self.pass(name),
                Err(e) => self.fail(
                    name,
                    &format!("expected error code {expected_code}, got {}", e.code()),
                ),
                Ok(v) => self.fail(name, &format!("expected error, got {v:?}")),
            }
        }

        fn summary(&self) {
            println!();
            println!("========================================");
            println!(
                "  {} passed, {} failed, {} total",
                self.passed,
                self.failed,
                self.passed + self.failed
            );
            if self.failed == 0 {
                println!("  ALL TESTS PASSED");
            } else {
                println!("  SOME TESTS FAILED");
            }
            println!("========================================");
        }
    }

    // ---------------------------------------------------------------
    // Slave receive helper — abstracts v1/v2 differences
    // ---------------------------------------------------------------

    /// Arm the slave to receive, run `f`, drain the result, disarm.
    fn with_slave_receive<F>(slave: &mut I2cSlaveDriver, buf_size: usize, f: F) -> Vec<u8>
    where
        F: FnOnce(),
    {
        let received = Arc::new(Mutex::new(Vec::new()));
        let clone = received.clone();
        let callback = move |data: &[u8]| {
            if let Ok(mut buf) = clone.lock() {
                buf.extend_from_slice(data);
            }
        };

        #[cfg(not(any(
            esp_idf_i2c_enable_slave_driver_version_2,
            esp_idf_version_at_least_6_0_0
        )))]
        slave.receive(buf_size, callback).unwrap();
        #[cfg(any(
            esp_idf_i2c_enable_slave_driver_version_2,
            esp_idf_version_at_least_6_0_0
        ))]
        slave.subscribe(callback).unwrap();

        f();
        let data = received.lock().unwrap().drain(..).collect();

        #[cfg(not(any(
            esp_idf_i2c_enable_slave_driver_version_2,
            esp_idf_version_at_least_6_0_0
        )))]
        slave.cancel_receive().unwrap();
        #[cfg(any(
            esp_idf_i2c_enable_slave_driver_version_2,
            esp_idf_version_at_least_6_0_0
        ))]
        slave.unsubscribe().unwrap();

        data
    }

    // ---------------------------------------------------------------
    // Common tests (work on both v1 and v2)
    // ---------------------------------------------------------------

    fn test_probe_ack(t: &mut TestRunner, bus: &I2cBusDriver) {
        let result = bus.probe(SLAVE_ADDR, 100);
        t.assert_ok("probe: ACK at slave address", &result);
    }

    fn test_probe_nack(t: &mut TestRunner, bus: &I2cBusDriver) {
        let result = bus.probe(UNUSED_ADDR, 100);
        t.assert_err("probe: NACK at unused address", &result);
    }

    fn test_bus_reset(t: &mut TestRunner, bus: &I2cBusDriver) {
        let result = bus.reset();
        t.assert_ok("bus reset: succeeds", &result);
    }

    fn test_master_write_slave_receive<'a>(
        t: &mut TestRunner,
        master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
        slave: &mut I2cSlaveDriver,
    ) {
        let tx: [u8; 8] = [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF];
        let rx = with_slave_receive(slave, tx.len(), || {
            master.transmit(&tx).unwrap();
        });
        t.assert_eq("master write -> slave receive", &tx, &rx);
    }

    fn test_slave_transmit_master_read<'a>(
        t: &mut TestRunner,
        master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
        slave: &mut I2cSlaveDriver,
    ) {
        let tx: [u8; 8] = [0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10];
        let mut rx: [u8; 8] = [0; 8];

        let written = slave.transmit(&tx).unwrap();
        master.receive(&mut rx).unwrap();
        t.assert_eq("slave transmit -> master read", &tx, &rx);

        if written == tx.len() {
            t.pass("slave transmit: write_len matches");
        } else {
            t.fail(
                "slave transmit: write_len matches",
                &format!("expected {}, got {written}", tx.len()),
            );
        }
    }

    fn test_master_write_read<'a>(
        t: &mut TestRunner,
        master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
        slave: &mut I2cSlaveDriver,
    ) {
        let slave_tx: [u8; 4] = [0xAA, 0xBB, 0xCC, 0xDD];
        slave.transmit(&slave_tx).unwrap();

        let master_tx: [u8; 1] = [0x42];
        let mut master_rx: [u8; 4] = [0; 4];

        let _rx = with_slave_receive(slave, 8, || {
            master.transmit_receive(&master_tx, &mut master_rx).unwrap();
        });

        t.assert_eq("write_read: read data matches", &slave_tx, &master_rx);
    }

    fn test_transaction_write_read<'a>(
        t: &mut TestRunner,
        master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
        slave: &mut I2cSlaveDriver,
    ) {
        let slave_tx: [u8; 4] = [0x11, 0x22, 0x33, 0x44];
        slave.transmit(&slave_tx).unwrap();

        let write_buf: [u8; 1] = [0x10];
        let mut read_buf: [u8; 4] = [0; 4];

        let _rx = with_slave_receive(slave, 8, || {
            master
                .transaction(&mut [Operation::Write(&write_buf), Operation::Read(&mut read_buf)])
                .unwrap();
        });

        t.assert_eq(
            "transaction [Write, Read]: data matches",
            &slave_tx,
            &read_buf,
        );
    }

    fn test_transaction_unsupported<'a>(
        t: &mut TestRunner,
        master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
    ) {
        let mut buf1 = [0u8; 4];
        let mut buf2 = [0u8; 4];

        let result = master.transaction(&mut [
            Operation::Write(&[0x01]),
            Operation::Write(&[0x02]),
            Operation::Read(&mut buf1),
        ]);
        t.assert_err_code(
            "transaction [W, W, R]: returns NOT_SUPPORTED",
            &result,
            ESP_ERR_NOT_SUPPORTED,
        );

        let result =
            master.transaction(&mut [Operation::Read(&mut buf1), Operation::Write(&[0x01])]);
        t.assert_err_code(
            "transaction [R, W]: returns NOT_SUPPORTED",
            &result,
            ESP_ERR_NOT_SUPPORTED,
        );

        let result =
            master.transaction(&mut [Operation::Read(&mut buf1), Operation::Read(&mut buf2)]);
        t.assert_err_code(
            "transaction [R, R]: returns NOT_SUPPORTED",
            &result,
            ESP_ERR_NOT_SUPPORTED,
        );
    }

    fn test_transaction_single_ops<'a>(
        t: &mut TestRunner,
        master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
        slave: &mut I2cSlaveDriver,
    ) {
        // Single write
        let _rx = with_slave_receive(slave, 8, || {
            master
                .transaction(&mut [Operation::Write(&[0xAA])])
                .unwrap();
        });
        t.pass("transaction [Write]: succeeds");

        // Single read
        slave.transmit(&[0xBB]).unwrap();
        let mut buf = [0u8; 1];
        let result = master.transaction(&mut [Operation::Read(&mut buf)]);
        t.assert_ok("transaction [Read]: succeeds", &result);
        t.assert_eq("transaction [Read]: data matches", &[0xBB], &buf);

        // Empty
        let result = master.transaction(&mut []);
        t.assert_ok("transaction []: succeeds", &result);
    }

    fn test_single_byte_transfer<'a>(
        t: &mut TestRunner,
        master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
        slave: &mut I2cSlaveDriver,
    ) {
        let rx = with_slave_receive(slave, 1, || {
            master.transmit(&[0x42]).unwrap();
        });
        t.assert_eq("single byte: master write", &[0x42], &rx);

        slave.transmit(&[0x99]).unwrap();
        let mut rx = [0u8; 1];
        master.receive(&mut rx).unwrap();
        t.assert_eq("single byte: slave transmit", &[0x99], &rx);
    }

    fn test_large_transfer<'a>(
        t: &mut TestRunner,
        master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
        slave: &mut I2cSlaveDriver,
    ) {
        let tx: Vec<u8> = (0..128).collect();
        let rx = with_slave_receive(slave, tx.len(), || {
            master.transmit(&tx).unwrap();
        });
        t.assert_eq("large transfer: 128 bytes master -> slave", &tx, &rx);
    }

    fn test_receive_cycle<'a>(
        t: &mut TestRunner,
        master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
        slave: &mut I2cSlaveDriver,
    ) {
        let rx1 = with_slave_receive(slave, 4, || {
            master.transmit(&[0x01, 0x02, 0x03, 0x04]).unwrap();
        });
        t.assert_eq(
            "receive cycle: first receive",
            &[0x01, 0x02, 0x03, 0x04],
            &rx1,
        );

        let rx2 = with_slave_receive(slave, 4, || {
            master.transmit(&[0x05, 0x06, 0x07, 0x08]).unwrap();
        });
        t.assert_eq(
            "receive cycle: second receive after disarm",
            &[0x05, 0x06, 0x07, 0x08],
            &rx2,
        );
    }

    // ---------------------------------------------------------------
    // v2-only tests
    // ---------------------------------------------------------------

    #[cfg(any(
        esp_idf_i2c_enable_slave_driver_version_2,
        esp_idf_version_at_least_6_0_0
    ))]
    mod v2_tests {
        use super::*;

        fn slave_subscribe(slave: &mut I2cSlaveDriver) -> Arc<Mutex<Vec<u8>>> {
            let received = Arc::new(Mutex::new(Vec::new()));
            let clone = received.clone();
            slave
                .subscribe(move |data: &[u8]| {
                    if let Ok(mut buf) = clone.lock() {
                        buf.extend_from_slice(data);
                    }
                })
                .unwrap();
            received
        }

        fn drain(received: &Arc<Mutex<Vec<u8>>>) -> Vec<u8> {
            received.lock().unwrap().drain(..).collect()
        }

        pub fn test_resubscribe<'a>(
            t: &mut TestRunner,
            master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
            slave: &mut I2cSlaveDriver,
        ) {
            let received1 = slave_subscribe(slave);
            master.transmit(&[0xAA]).unwrap();
            let rx1 = drain(&received1);
            t.assert_eq("resubscribe: first callback receives data", &[0xAA], &rx1);

            // Resubscribe replaces the callback
            let received2 = slave_subscribe(slave);
            master.transmit(&[0xBB]).unwrap();
            let rx2 = drain(&received2);
            slave.unsubscribe().unwrap();

            t.assert_eq("resubscribe: second callback receives data", &[0xBB], &rx2);

            let stale = drain(&received1);
            if stale.is_empty() {
                t.pass("resubscribe: old callback did not fire after resubscribe");
            } else {
                t.fail(
                    "resubscribe: old callback did not fire after resubscribe",
                    &format!("old callback received {stale:02x?}"),
                );
            }
        }

        pub fn test_multiple_writes<'a>(
            t: &mut TestRunner,
            master: &mut I2cDriver<'a, &'a I2cBusDriver<'a>>,
            slave: &mut I2cSlaveDriver,
        ) {
            let received = slave_subscribe(slave);

            master.transmit(&[0x01, 0x02]).unwrap();
            master.transmit(&[0x03, 0x04]).unwrap();
            master.transmit(&[0x05, 0x06]).unwrap();

            let rx = drain(&received);
            slave.unsubscribe().unwrap();

            t.assert_eq(
                "subscribe: accumulates multiple master writes",
                &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
                &rx,
            );
        }
    }

    // ---------------------------------------------------------------
    // Entry point
    // ---------------------------------------------------------------

    pub fn main() -> anyhow::Result<()> {
        esp_idf_hal::sys::link_patches();

        println!("========================================");
        println!("  I2C master/slave hardware tests");
        println!("========================================");
        println!();

        let peripherals = Peripherals::take()?;

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
        let mut slave = I2cSlaveDriver::new(
            peripherals.i2c1,
            peripherals.pins.gpio18,
            peripherals.pins.gpio19,
            SLAVE_ADDR,
            &I2cSlaveConfig::new().send_buf_depth(256),
        )?;

        let mut t = TestRunner::new();

        println!("[bus]");
        test_probe_ack(&mut t, &bus);
        test_probe_nack(&mut t, &bus);
        test_bus_reset(&mut t, &bus);

        println!("[master write / slave receive]");
        test_master_write_slave_receive(&mut t, &mut master, &mut slave);
        test_single_byte_transfer(&mut t, &mut master, &mut slave);
        test_large_transfer(&mut t, &mut master, &mut slave);

        println!("[slave transmit / master read]");
        test_slave_transmit_master_read(&mut t, &mut master, &mut slave);

        println!("[master write_read]");
        test_master_write_read(&mut t, &mut master, &mut slave);

        println!("[transaction]");
        test_transaction_single_ops(&mut t, &mut master, &mut slave);
        test_transaction_write_read(&mut t, &mut master, &mut slave);
        test_transaction_unsupported(&mut t, &mut master);

        println!("[receive lifecycle]");
        test_receive_cycle(&mut t, &mut master, &mut slave);

        #[cfg(any(
            esp_idf_i2c_enable_slave_driver_version_2,
            esp_idf_version_at_least_6_0_0
        ))]
        {
            println!("[v2 subscribe]");
            v2_tests::test_resubscribe(&mut t, &mut master, &mut slave);
            v2_tests::test_multiple_writes(&mut t, &mut master, &mut slave);
        }

        t.summary();

        loop {
            FreeRtos::delay_ms(1000);
        }
    }
}
