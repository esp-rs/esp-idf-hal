//! Simple HTTP client example.

use embedded_svc::{
    http::{client::Client as HttpClient, Method},
    io::Write,
    utils::io,
    wifi::{AuthMethod, ClientConfiguration, Configuration},
};

use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::http::client::EspHttpConnection;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition};

use log::{error, info};

const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASS");

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    EspLogger::initialize_default();

    // Setup Wifi

    let peripherals = Peripherals::take().unwrap();
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
        sys_loop,
    )?;

    connect_wifi(&mut wifi)?;

    // Create HTTP(S) client
    let mut client = HttpClient::wrap(EspHttpConnection::new(&Default::default())?);

    // GET
    get_request(&mut client)?;

    // POST
    post_request(&mut client)?;

    Ok(())
}

/// Send a HTTP GET request.
fn get_request(client: &mut HttpClient<EspHttpConnection>) -> anyhow::Result<()> {
    // Prepare headers and URL
    let headers = [("accept", "text/plain"), ("connection", "close")];
    let url = "http://ifconfig.net/";

    // Send request
    //
    // Note: If you don't want to pass in any headers, you can also use `client.get(url, headers)`.
    let request = client.request(Method::Get, url, &headers)?;
    info!("-> GET {}", url);
    let mut response = request.submit()?;

    // Process response
    let status = response.status();
    info!("<- {}", status);
    let (_headers, mut body) = response.split();
    let mut buf = [0u8; 1024];
    let bytes_read = io::try_read_full(&mut body, &mut buf).map_err(|e| e.0)?;
    info!("Read {} bytes", bytes_read);
    match std::str::from_utf8(&buf[0..bytes_read]) {
        Ok(body_string) => info!(
            "Response body (truncated to {} bytes): {:?}",
            buf.len(),
            body_string
        ),
        Err(e) => error!("Error decoding response body: {}", e),
    };

    // Drain the remaining response bytes
    while body.read(&mut buf)? > 0 {}

    Ok(())
}

/// Send a HTTP POST request.
fn post_request(client: &mut HttpClient<EspHttpConnection>) -> anyhow::Result<()> {
    // Prepare payload
    let payload = b"Hello world!";

    // Prepare headers and URL
    let content_length_header = format!("{}", payload.len());
    let headers = [
        ("accept", "text/plain"),
        ("content-type", "text/plain"),
        ("connection", "close"),
        ("content-length", &*content_length_header),
    ];
    let url = "http://example.org/";

    // Send request
    let mut request = client.post(url, &headers)?;
    request.write_all(payload)?;
    request.flush()?;
    info!("-> POST {}", url);
    let mut response = request.submit()?;

    // Process response
    let status = response.status();
    info!("<- {}", status);
    let (_headers, mut body) = response.split();
    let mut buf = [0u8; 1024];
    let bytes_read = io::try_read_full(&mut body, &mut buf).map_err(|e| e.0)?;
    info!("Read {} bytes", bytes_read);
    match std::str::from_utf8(&buf[0..bytes_read]) {
        Ok(body_string) => info!(
            "Response body (truncated to {} bytes): {:?}",
            buf.len(),
            body_string
        ),
        Err(e) => error!("Error decoding response body: {}", e),
    };

    // Drain the remaining response bytes
    while body.read(&mut buf)? > 0 {}

    Ok(())
}

fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<()> {
    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        bssid: None,
        auth_method: AuthMethod::WPA2Personal,
        password: PASSWORD.into(),
        channel: None,
    });

    wifi.set_configuration(&wifi_configuration)?;

    wifi.start()?;
    info!("Wifi started");

    wifi.connect()?;
    info!("Wifi connected");

    wifi.wait_netif_up()?;
    info!("Wifi netif up");

    Ok(())
}
