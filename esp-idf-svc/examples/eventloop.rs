use core::ffi::CStr;

use esp_idf_svc::eventloop::*;

#[derive(Copy, Clone, Debug)]
enum CustomEvent {
    Start,
    Tick(u32),
}

impl EspEventSource for CustomEvent {
    fn source() -> &'static CStr {
        CStr::from_bytes_with_nul(b"DEMO-SERVICE\0").unwrap()
    }
}

impl EspEventSerializer for CustomEvent {
    type Data<'a> = CustomEvent;

    fn serialize<'a, F>(event: &'a Self::Data<'a>, f: F) -> R {
        f(&unsafe { EspEventPostData::new(Self::source(), Self::event_id(), event) })
    }
}

impl EspTypedEventDeserializer<EventLoopMessage> for EventLoopMessage {
    fn deserialize<R>(
        data: &EspEventFetchData,
        f: &mut impl for<'a> FnMut(&'a EventLoopMessage) -> R,
    ) -> R {
        f(unsafe { data.as_payload() })
    }
}

fn test_eventloop() -> Result<(EspBackgroundEventLoop, EspBackgroundSubscription<'static>)> {
    info!("About to start a background event loop");
    let eventloop = EspBackgroundEventLoop::new(&Default::default())?;

    info!("About to subscribe to the background event loop");
    let subscription = eventloop.subscribe(|message: &EventLoopMessage| {
        info!("Got message from the event loop: {:?}", message.0);
    })?;

    Ok((eventloop, subscription))
}
