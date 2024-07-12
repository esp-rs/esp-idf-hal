use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::onewire::BusDriver;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::{
    FixedLengthSignal, PinState, Pulse, PulseTicks, Receive, RmtReceiveConfig, RmtTransmitConfig,
    RxRmtDriver, TxRmtDriver,
};

fn main() -> anyhow::Result<()> {
    println!("Starting APP!");

    let peripherals = Peripherals::take()?;

    let onewire_pin = peripherals.pins.gpio16;

    let mut rmt_onewire = BusDriver::new(onewire_pin)?;

    let mut search = rmt_onewire.search()?;

    let search_device = search.next_device()?;

    println!("Found Device: {}", search_device.address());
    loop {
        FreeRtos::delay_ms(3000);
    }
}
