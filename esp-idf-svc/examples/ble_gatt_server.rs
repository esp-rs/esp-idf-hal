//! Example of a BLE GATT server using the ESP IDF NimBLE bindings.
//!
//! Requires a NimBLE-enabled build.

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

#[cfg(all(not(any(esp32s2, esp32p4)), esp_idf_bt_nimble_enabled))]
fn main() -> anyhow::Result<()> {
    example::main()
}

#[cfg(not(all(not(any(esp32s2, esp32p4)), esp_idf_bt_nimble_enabled)))]
fn main() -> anyhow::Result<()> {
    panic!("This example requires a NimBLE-enabled build (CONFIG_BT_NIMBLE_ENABLED=y) on a chip with a BLE radio");
}

#[cfg(all(not(any(esp32s2, esp32p4)), esp_idf_bt_nimble_enabled))]
mod example {
    use core::sync::atomic::{AtomicU16, Ordering};
    use std::sync::Mutex;

    use esp_idf_svc::ble::gap::{self, BleAdvFields, BleGapEvent};
    use esp_idf_svc::ble::gatt::gatts::{
        self, BleGattAccess, BleGattCharacteristic, BleGattRegister, BleGattService,
        BleGattServices, ConnectionId, GattsSetup,
    };
    use esp_idf_svc::ble::gatt::BleGattCharFlag;
    use esp_idf_svc::ble::{ensure_addr, BleError, BleSetup, BleUuid};
    use esp_idf_svc::hal::delay::FreeRtos;
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::log::EspLogger;

    use enumset::enum_set;
    use log::{info, warn};

    const DEVICE_NAME: &str = "esp-nimble";

    // Our service UUID
    pub const SERVICE_UUID: u128 = 0xad91b201734740479e173bed82d75f9d;

    /// Our "recv" characteristic - i.e. where clients can send data.
    pub const RECV_CHARACTERISTIC_UUID: u128 = 0xb6fccb5087be44f3ae22f85485ea42c4;
    /// Our "indicate" characteristic - i.e. where clients can receive data if they subscribe to it
    pub const IND_CHARACTERISTIC_UUID: u128 = 0x503de214868246c4828fd59144da41be;

    // Server state. We capture the indicate characteristic's value handle from the
    // registration callback (see `on_gatts_register` below); a real server tracking
    // several handles would keep a uuid -> handle map instead of a single slot.
    static SUBSCRIBERS: Mutex<Vec<ConnectionId>> = Mutex::new(Vec::new());
    static IND_VAL_HANDLE: AtomicU16 = AtomicU16::new(0);

    pub fn main() -> anyhow::Result<()> {
        esp_idf_svc::sys::link_patches();
        EspLogger::initialize_default();

        let peripherals = Peripherals::take()?;

        let services = BleGattServices::new(vec![BleGattService::new(
            true,
            BleUuid::uuid128(SERVICE_UUID),
            vec![
                // "recv": clients write here; we just log what arrives.
                BleGattCharacteristic::new(
                    BleUuid::uuid128(RECV_CHARACTERISTIC_UUID),
                    enum_set!(BleGattCharFlag::Write),
                    |access| {
                        if let BleGattAccess::Write { data, .. } = access {
                            let mut buf = [0u8; 200];
                            match data.read(&mut buf) {
                                Ok(n) => info!("recv {n} bytes: {:?}", &buf[..n]),
                                Err(e) => warn!("recv read failed: {e}"),
                            }
                        }
                        0
                    },
                ),
                // "indicate": clients subscribe and get the counter pushed from the loop
                // below. NimBLE adds the CCCD (0x2902) automatically for this flag, so
                // there is no descriptor to declare and no read/write to service here.
                BleGattCharacteristic::new(
                    BleUuid::uuid128(IND_CHARACTERISTIC_UUID),
                    enum_set!(BleGattCharFlag::Indicate),
                    |_access| 0,
                ),
            ],
        )]);

        let mut setup = BleSetup::new(peripherals.modem)?;

        GattsSetup::new(&mut setup).add_services(&services)?;

        // NimBLE assigns attribute handles during registration and reports them here,
        // on the host task. We stash the indicate handle so the loop below can push to
        // it; matching on the UUID is how we tell our characteristics apart.
        setup.on_gatts_register(|event| {
            if let BleGattRegister::Characteristic {
                uuid, val_handle, ..
            } = event
            {
                if uuid == BleUuid::uuid128(IND_CHARACTERISTIC_UUID) {
                    IND_VAL_HANDLE.store(val_handle, Ordering::Relaxed);
                }
            }
        });

        // We wait until the stack is "in sync" before we can start using it. Note this
        // closure needs to handle being called multiple times in case the stack resets.
        setup.on_sync(|| match start_advertising() {
            Ok(()) => info!("advertising as {DEVICE_NAME:?}"),
            Err(e) => warn!("failed to start advertising: {e}"),
        });

        setup.on_gap_event(|event| {
            match event {
                BleGapEvent::Connect {
                    conn_handle,
                    status,
                } => info!("connected (handle {conn_handle}): {status:?}"),
                BleGapEvent::Disconnect {
                    conn_handle,
                    reason,
                } => {
                    info!("disconnected ({reason}); restarting advertising");
                    SUBSCRIBERS.lock().unwrap().retain(|&c| c != conn_handle);
                    if let Err(e) = start_advertising() {
                        warn!("failed to restart advertising: {e}");
                    }
                }
                BleGapEvent::Subscribe {
                    conn_handle,
                    attr_handle,
                    cur_indicate,
                    ..
                } => {
                    if attr_handle == IND_VAL_HANDLE.load(Ordering::Relaxed) {
                        let mut subs = SUBSCRIBERS.lock().unwrap();
                        subs.retain(|&c| c != conn_handle);
                        if cur_indicate {
                            let _ = subs.push(conn_handle);
                        }
                    }
                }
                _ => {}
            }

            0
        });

        let _driver = setup.start()?;
        info!("NimBLE host started");

        let mut counter: u16 = 0;
        loop {
            FreeRtos::delay_ms(1000);

            // The handle stays 0 until the GATT registration callback sets it to
            // whatver val_handle NimBLE assigned.
            let ind_handle = IND_VAL_HANDLE.load(Ordering::Relaxed);
            if ind_handle == 0 {
                continue;
            }

            counter = counter.wrapping_add(1);

            // Copy the subscriber list out so the lock isn't held across `indicate`.
            let subs = SUBSCRIBERS.lock().unwrap().clone();
            for conn in subs {
                if let Err(e) = gatts::indicate(conn, ind_handle, &counter.to_le_bytes()) {
                    warn!("indicate to {conn} failed: {e}");
                }
            }
        }
    }

    /// Configure and start a connectable legacy advertisement
    /// n.b. NimBLE exposes a mutually-exclusive "extended" advertisement API as well
    /// if you set the right build flags
    fn start_advertising() -> Result<(), BleError> {
        use esp_idf_svc::ble::gap::BleAdvParams;

        ensure_addr(false)?;
        gap::svc_set_device_name(DEVICE_NAME)?;

        let fields = BleAdvFields {
            flags: 0x06, // LE General Discoverable, BR/EDR unsupported
            name: Some(DEVICE_NAME),
            ..Default::default()
        };
        gap::adv_set_fields(&fields)?;

        let params = BleAdvParams {
            conn_mode: 2,   // BLE_GAP_CONN_MODE_UND
            disc_mode: 2,   // BLE_GAP_DISC_MODE_GEN
            itvl_min: 0x30, // 30 ms, in 0.625 ms units
            itvl_max: 0x60, // 60 ms
            ..Default::default()
        };
        gap::adv_start(0 /* BLE_OWN_ADDR_PUBLIC */, &params)
    }
}
