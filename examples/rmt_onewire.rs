use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::{
    FixedLengthSignal, PinState, Pulse, PulseTicks, Receive, RmtReceiveConfig, RmtTransmitConfig,
    RxRmtDriver, TxRmtDriver,
};

fn main() -> anyhow::Result<()> {
    println!("Starting APP!");

    let peripherals = Peripherals::take()?;

    loop {
        FreeRtos::delay_ms(3000);
    }
}
