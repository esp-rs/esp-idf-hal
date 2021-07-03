#[cfg(test)]
mod test {
    use std::{any::Any, borrow::Borrow, sync::Arc};

    use embedded_svc::{
        edge_config::wifi,
        httpd::{app, registry::Registry, sessions, StateMap},
        wifi::{Configuration, Wifi},
    };

    use crate::{
        httpd::ServerRegistry, netif::EspNetif, nvs::EspDefaultNvs, sysloop::EspSysLoop,
        wifi::EspWifi,
    };

    #[test]
    pub fn test() -> anyhow::Result<()> {
        let mut wifi = EspWifi::new(
            Arc::new(EspNetif::new()?),
            Arc::new(EspSysLoop::new()?),
            Arc::new(EspDefaultNvs::new()?),
        )?;

        wifi.set_configuration(&Configuration::AccessPoint(Default::default()))?;

        let boxed: Box<dyn Any> = Box::new(wifi);

        let app: StateMap = vec![("wifi".to_string(), boxed)].into_iter().collect();

        let _server = ServerRegistry::new()
            .register(|registry| wifi::register(registry, "/api", None))?
            .at("")
            .middleware(sessions::middleware(Default::default()))?
            .at("")
            .middleware(app::middleware(app))?
            .start(&Default::default())?;

        Ok(())
    }
}
