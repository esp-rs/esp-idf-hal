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

    use embedded_hal::i2c::I2c as I2cTrait;

    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::i2c::*;
    use esp_idf_hal::peripherals::Peripherals;
    use esp_idf_hal::sys::ESP_ERR_NOT_SUPPORTED;

    const SLAVE_ADDR: u8 = 0x22;
    const UNUSED_ADDR: u8 = 0x7E;

    type Master<'a> = I2cDriver<'a, &'a I2cBusDriver<'a>>;

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

        fn assert_err<T: core::fmt::Debug, E>(&mut self, name: &str, result: &Result<T, E>) {
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

    /// Master writes `tx`, slave receives — assert round-trip matches.
    fn assert_master_write<'a>(
        t: &mut TestRunner,
        name: &str,
        master: &mut Master<'a>,
        slave: &mut I2cSlaveDriver,
        tx: &[u8],
    ) {
        let rx = with_slave_receive(slave, tx.len(), || {
            master.transmit(tx).unwrap();
        });
        t.assert_eq(name, tx, &rx);
    }

    /// Slave loads `tx` into TX buffer, master reads — assert round-trip matches.
    fn assert_slave_transmit<'a>(
        t: &mut TestRunner,
        name: &str,
        master: &mut Master<'a>,
        slave: &mut I2cSlaveDriver,
        tx: &[u8],
    ) {
        slave.transmit(tx).unwrap();
        let mut rx = vec![0u8; tx.len()];
        master.receive(&mut rx).unwrap();
        t.assert_eq(name, tx, &rx);
    }

    // ---------------------------------------------------------------
    // Tests
    // ---------------------------------------------------------------

    fn test_bus<'a>(t: &mut TestRunner, bus: &I2cBusDriver<'a>) {
        t.assert_ok("probe: ACK at slave address", &bus.probe(SLAVE_ADDR, 100));
        t.assert_err(
            "probe: NACK at unused address",
            &bus.probe(UNUSED_ADDR, 100),
        );
        t.assert_ok("bus reset", &bus.reset());
    }

    fn test_master_write_slave_receive<'a>(
        t: &mut TestRunner,
        master: &mut Master<'a>,
        slave: &mut I2cSlaveDriver,
    ) {
        assert_master_write(
            t,
            "8 bytes",
            master,
            slave,
            &[0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF],
        );
        assert_master_write(t, "1 byte", master, slave, &[0x42]);

        let large: Vec<u8> = (0..128).collect();
        assert_master_write(t, "128 bytes", master, slave, &large);
    }

    fn test_slave_transmit_master_read<'a>(
        t: &mut TestRunner,
        master: &mut Master<'a>,
        slave: &mut I2cSlaveDriver,
    ) {
        assert_slave_transmit(
            t,
            "8 bytes",
            master,
            slave,
            &[0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10],
        );
        assert_slave_transmit(t, "1 byte", master, slave, &[0x99]);
    }

    fn test_write_read<'a>(
        t: &mut TestRunner,
        master: &mut Master<'a>,
        slave: &mut I2cSlaveDriver,
    ) {
        let slave_tx = [0xAA, 0xBB, 0xCC, 0xDD];
        slave.transmit(&slave_tx).unwrap();

        let mut master_rx = [0u8; 4];
        let _rx = with_slave_receive(slave, 8, || {
            master.transmit_receive(&[0x42], &mut master_rx).unwrap();
        });
        t.assert_eq("transmit_receive", &slave_tx, &master_rx);

        // Same via transaction API
        let slave_tx = [0x11, 0x22, 0x33, 0x44];
        slave.transmit(&slave_tx).unwrap();

        let mut read_buf = [0u8; 4];
        let _rx = with_slave_receive(slave, 8, || {
            master
                .transaction(&mut [Operation::Write(&[0x10]), Operation::Read(&mut read_buf)])
                .unwrap();
        });
        t.assert_eq("transaction [Write, Read]", &slave_tx, &read_buf);
    }

    fn test_transaction_single_ops<'a>(
        t: &mut TestRunner,
        master: &mut Master<'a>,
        slave: &mut I2cSlaveDriver,
    ) {
        assert_master_write(t, "transaction [Write]", master, slave, &[0xAA]);
        assert_slave_transmit(t, "transaction [Read]", master, slave, &[0xBB]);
        t.assert_ok("transaction []", &master.transaction(&mut []));
    }

    fn test_transaction_unsupported<'a>(t: &mut TestRunner, master: &mut Master<'a>) {
        let mut buf1 = [0u8; 4];
        let mut buf2 = [0u8; 4];

        let result = master.transaction(&mut [
            Operation::Write(&[0x01]),
            Operation::Write(&[0x02]),
            Operation::Read(&mut buf1),
        ]);
        t.assert_err_code("[W, W, R]", &result, ESP_ERR_NOT_SUPPORTED);

        let result =
            master.transaction(&mut [Operation::Read(&mut buf1), Operation::Write(&[0x01])]);
        t.assert_err_code("[R, W]", &result, ESP_ERR_NOT_SUPPORTED);

        let result =
            master.transaction(&mut [Operation::Read(&mut buf1), Operation::Read(&mut buf2)]);
        t.assert_err_code("[R, R]", &result, ESP_ERR_NOT_SUPPORTED);

        let result =
            master.transaction(&mut [Operation::Write(&[0x01]), Operation::Write(&[0x02])]);
        t.assert_err_code("[W, W]", &result, ESP_ERR_NOT_SUPPORTED);
    }

    fn test_receive_lifecycle<'a>(
        t: &mut TestRunner,
        master: &mut Master<'a>,
        slave: &mut I2cSlaveDriver,
    ) {
        // arm -> use -> disarm -> arm again
        assert_master_write(t, "first arm", master, slave, &[0x01, 0x02, 0x03, 0x04]);
        assert_master_write(
            t,
            "re-arm after disarm",
            master,
            slave,
            &[0x05, 0x06, 0x07, 0x08],
        );

        // disarm when idle is a no-op
        #[cfg(not(any(
            esp_idf_i2c_enable_slave_driver_version_2,
            esp_idf_version_at_least_6_0_0
        )))]
        let result = slave.cancel_receive();
        #[cfg(any(
            esp_idf_i2c_enable_slave_driver_version_2,
            esp_idf_version_at_least_6_0_0
        ))]
        let result = slave.unsubscribe();
        t.assert_ok("disarm when idle", &result);
    }

    fn test_embedded_hal_trait<'a>(
        t: &mut TestRunner,
        master: &mut Master<'a>,
        slave: &mut I2cSlaveDriver,
    ) {
        // write
        let tx = [0xDE, 0xAD];
        let rx = with_slave_receive(slave, tx.len(), || {
            I2cTrait::write(master, SLAVE_ADDR, &tx).unwrap();
        });
        t.assert_eq("trait write", &tx, &rx);

        // read
        slave.transmit(&[0xBE, 0xEF]).unwrap();
        let mut rx = [0u8; 2];
        I2cTrait::read(master, SLAVE_ADDR, &mut rx).unwrap();
        t.assert_eq("trait read", &[0xBE, 0xEF], &rx);

        // write_read
        let slave_tx = [0xCA, 0xFE];
        slave.transmit(&slave_tx).unwrap();
        let mut master_rx = [0u8; 2];
        let _rx = with_slave_receive(slave, 8, || {
            I2cTrait::write_read(master, SLAVE_ADDR, &[0x01], &mut master_rx).unwrap();
        });
        t.assert_eq("trait write_read", &slave_tx, &master_rx);

        // wrong address
        t.assert_err(
            "wrong addr: write",
            &I2cTrait::write(master, UNUSED_ADDR, &[0x00]),
        );
        let mut buf = [0u8; 1];
        t.assert_err(
            "wrong addr: read",
            &I2cTrait::read(master, UNUSED_ADDR, &mut buf),
        );
        t.assert_err(
            "wrong addr: write_read",
            &I2cTrait::write_read(master, UNUSED_ADDR, &[0x00], &mut buf),
        );
        t.assert_err(
            "wrong addr: transaction",
            &I2cTrait::transaction(master, UNUSED_ADDR, &mut [Operation::Write(&[0x00])]),
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

        pub fn test_subscribe<'a>(
            t: &mut TestRunner,
            master: &mut Master<'a>,
            slave: &mut I2cSlaveDriver,
        ) {
            // resubscribe replaces callback
            let received1 = slave_subscribe(slave);
            master.transmit(&[0xAA]).unwrap();
            t.assert_eq("first callback", &[0xAA], &drain(&received1));

            let received2 = slave_subscribe(slave);
            master.transmit(&[0xBB]).unwrap();
            t.assert_eq(
                "second callback after resubscribe",
                &[0xBB],
                &drain(&received2),
            );

            slave.unsubscribe().unwrap();
            if drain(&received1).is_empty() {
                t.pass("old callback silent after resubscribe");
            } else {
                t.fail(
                    "old callback silent after resubscribe",
                    "received stale data",
                );
            }

            // multiple writes accumulate
            let received = slave_subscribe(slave);
            master.transmit(&[0x01, 0x02]).unwrap();
            master.transmit(&[0x03, 0x04]).unwrap();
            master.transmit(&[0x05, 0x06]).unwrap();
            slave.unsubscribe().unwrap();
            t.assert_eq(
                "accumulates multiple writes",
                &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
                &drain(&received),
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
        test_bus(&mut t, &bus);

        println!("[master write -> slave receive]");
        test_master_write_slave_receive(&mut t, &mut master, &mut slave);

        println!("[slave transmit -> master read]");
        test_slave_transmit_master_read(&mut t, &mut master, &mut slave);

        println!("[write_read]");
        test_write_read(&mut t, &mut master, &mut slave);

        println!("[transaction]");
        test_transaction_single_ops(&mut t, &mut master, &mut slave);
        test_transaction_unsupported(&mut t, &mut master);

        println!("[receive lifecycle]");
        test_receive_lifecycle(&mut t, &mut master, &mut slave);

        println!("[embedded-hal trait]");
        test_embedded_hal_trait(&mut t, &mut master, &mut slave);

        #[cfg(any(
            esp_idf_i2c_enable_slave_driver_version_2,
            esp_idf_version_at_least_6_0_0
        ))]
        {
            println!("[v2 subscribe]");
            v2_tests::test_subscribe(&mut t, &mut master, &mut slave);
        }

        t.summary();

        loop {
            FreeRtos::delay_ms(1000);
        }
    }
}
