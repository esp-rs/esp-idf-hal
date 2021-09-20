#[cfg(test)]
mod test {
    use std::sync::{Arc, Mutex};

    use embedded_svc::{
        edge_config::wifi,
        httpd::{
            app,
            registry::Registry,
            sessions::{self, Sessions},
            StateMap,
        },
        wifi::{Configuration, Wifi},
    };

    use crate::{
        httpd::ServerRegistry, netif::EspNetifStack, nvs::EspDefaultNvs, sysloop::EspSysLoopStack,
        wifi::EspWifi,
    };

    #[test]
    pub fn test() -> anyhow::Result<()> {
        let mut wifi = EspWifi::new(
            Arc::new(EspNetifStack::new()?),
            Arc::new(EspSysLoopStack::new()?),
            Arc::new(EspDefaultNvs::new()?),
        )?;

        wifi.set_configuration(&Configuration::AccessPoint(Default::default()))?;

        let wifi = Arc::new(Mutex::new(wifi));

        let app: StateMap = vec![].into_iter().collect();

        let _server = ServerRegistry::new()
            .register(|registry| wifi::register(registry, "/api", wifi, None))?
            .at("")
            .middleware(sessions::middleware(Sessions::new(10, || [0u8; 16])))?
            .at("")
            .middleware(app::middleware(app))?
            .start(&Default::default())?;

        Ok(())
    }
}
