use std::sync::Arc;

use embedded_svc::edge_config::*;
//use embedded_svc::httpd::registry::Registry;

use crate::netif::EspNetif;
use crate::sysloop::EspSysLoop;
use crate::nvs::EspDefaultNvs;
//use crate::httpd;

pub struct App {
    wifi: crate::wifi::EspWifi,
}

impl App {
    pub fn new() -> Self {
        App {
            wifi: crate::wifi::EspWifi::new(
                Arc::new(EspNetif::new().unwrap()),
                Arc::new(EspSysLoop::new().unwrap()),
                Arc::new(EspDefaultNvs::new().unwrap())).unwrap(),
        }
    }
}

impl wifi::AsWifi<crate::wifi::EspWifi> for App {
    fn as_wifi(&self) -> &crate::wifi::EspWifi {
        &self.wifi
    }

    fn as_wifi_mut(&mut self) -> &mut crate::wifi::EspWifi {
        &mut self.wifi
    }
}

pub struct Session {
    wifi_session: wifi::WifiSession,
}

impl wifi::AsWifiSession for Session {
    fn as_wifi_session(&self) -> &wifi::WifiSession {
        &self.wifi_session
    }

    fn as_wifi_session_mut(&mut self) -> &mut wifi::WifiSession {
        &mut self.wifi_session
    }
}

impl AsRole for Session {
    fn as_role(&self) -> Role {
        Role::Admin
    }
}

#[test]
pub fn test() -> anyhow::Result<()> {
    let app = Arc::new(RwLock::new(App::new()));

    let mut server: httpd::Server<Session, _> = httpd::Server::default_new(app)?;

    //let _server = wifi::register(server, "", Role::None)?;
    server.register_all(wifi::get_registrations("", Role::None))?;

    Ok(())
}
