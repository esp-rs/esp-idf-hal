use core::time::Duration;

use embassy_futures::select::{select, Either};

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::mqtt::client::*;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::EspError;
use esp_idf_svc::timer::{EspAsyncTimer, EspTaskTimerService, EspTimerService};
use esp_idf_svc::wifi::*;

use log::*;

const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASS");

const MQTT_URL: &str = "mqtt://broker.emqx.io:1883";
const MQTT_CLIENT_ID: &str = "esp-mqtt-demo";
const MQTT_TOPIC: &str = "esp-mqtt-demo";

fn main() -> Result<(), EspError> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    esp_idf_svc::hal::task::block_on(async {
        let timer_service = EspTimerService::new()?;
        let _wifi = wifi_create(&timer_service).await?;

        let (mut client, mut conn) = mqtt_create(MQTT_URL, MQTT_CLIENT_ID)?;

        let mut timer = timer_service.timer_async()?;
        run(&mut client, &mut conn, &mut timer, MQTT_TOPIC).await
    })
}

async fn run(
    client: &mut EspMqttClient<'_>,
    connection: &mut EspMqttConnection,
    timer: &mut EspAsyncTimer,
    topic: &str,
) -> Result<(), EspError> {
    info!("About to start the MQTT client");

    info!("MQTT client started");

    let res = select(
        async move {
            info!("MQTT Listening for messages");

            while let Ok(event) = connection.next_async().await {
                info!("[Queue] Event: {}", event.payload());
            }

            info!("Connection closed");

            Ok(())
        },
        async move {
            client.subscribe(topic, QoS::AtMostOnce)?;

            info!("Subscribed to topic \"{topic}\"");

            // Just to give a chance of our connection to get even the first published message
            timer.after(Duration::from_millis(500)).await?;

            let payload = "Hello from esp-mqtt-demo!";

            loop {
                client.enqueue(topic, QoS::AtMostOnce, false, payload.as_bytes())?;

                info!("Published \"{payload}\" to topic \"{topic}\"");

                let sleep_secs = 2;

                info!("Now sleeping for {sleep_secs}s...");
                timer.after(Duration::from_secs(sleep_secs)).await?;
            }
        },
    )
    .await;

    match res {
        Either::First(res) => res,
        Either::Second(res) => res,
    }
}

fn mqtt_create(
    url: &str,
    client_id: &str,
) -> Result<(EspMqttClient<'static>, EspMqttConnection), EspError> {
    let (mqtt_client, mqtt_conn) = EspMqttClient::new_with_conn(
        url,
        &MqttClientConfiguration {
            client_id: Some(client_id),
            ..Default::default()
        },
    )?;

    Ok((mqtt_client, mqtt_conn))
}

async fn wifi_create(timer_service: &EspTaskTimerService) -> Result<EspWifi<'static>, EspError> {
    let peripherals = Peripherals::take()?;

    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let mut esp_wifi = EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?;

    let mut wifi = AsyncWifi::wrap(&mut esp_wifi, sys_loop, timer_service.clone())?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    }))?;

    wifi.start().await?;
    wifi.wait_netif_up().await?;

    info!(
        "Created Wi-Fi with WIFI_SSID `{}` and WIFI_PASS `{}`",
        SSID, PASSWORD
    );

    Ok(esp_wifi)
}
