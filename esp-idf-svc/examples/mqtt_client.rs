use core::time::Duration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::mqtt::client::*;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::EspError;
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

    let _wifi = wifi_create()?;

    let (mut client, mut conn) = mqtt_create(MQTT_URL, MQTT_CLIENT_ID)?;

    run(&mut client, &mut conn, MQTT_TOPIC)
}

fn run(
    client: &mut EspMqttClient<'_>,
    connection: &mut EspMqttConnection,
    topic: &str,
) -> Result<(), EspError> {
    std::thread::scope(|s| {
        info!("About to start the MQTT client");

        info!("MQTT client started");

        s.spawn(move || {
            info!("MQTT Listening for messages");

            while let Ok(event) = connection.next() {
                info!("[Queue] Event: {}", event.payload());
            }

            info!("Connection closed");
        });

        client.subscribe(topic, QoS::AtMostOnce)?;

        info!("Subscribed to topic \"{topic}\"");

        // Just to give a chance of our connection to get even the first published message
        std::thread::sleep(Duration::from_millis(500));

        let payload = "Hello from esp-mqtt-demo!";

        loop {
            client.enqueue(topic, QoS::AtMostOnce, false, payload.as_bytes())?;

            info!("Published \"{payload}\" to topic \"{topic}\"");

            let sleep_secs = 2;

            info!("Now sleeping for {sleep_secs}s...");
            std::thread::sleep(Duration::from_secs(sleep_secs));
        }
    })
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

fn wifi_create() -> Result<EspWifi<'static>, EspError> {
    let peripherals = Peripherals::take()?;

    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let mut esp_wifi = EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?;

    let mut wifi = BlockingWifi::wrap(&mut esp_wifi, sys_loop)?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    }))?;

    wifi.start()?;
    wifi.wait_netif_up()?;

    info!(
        "Created Wi-Fi with WIFI_SSID `{}` and WIFI_PASS `{}`",
        SSID, PASSWORD
    );

    Ok(esp_wifi)
}
