use esp_idf_hal::sys::{EspError, TaskHandle_t}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

fn main() -> Result<(), EspError> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_hal::sys::link_patches();

    let per = esp_idf_hal::peripherals::Peripherals::take().unwrap();

    // This handle will be used as the address for the event in the callback.
    // Make sure that the handle / thread lives always longer as the callback it is used in
    let main_task_handle: TaskHandle_t = esp_idf_hal::task::current().unwrap();

    // BaseClock for the Timer is the APB_CLK that is running on 80MHz at default
    // The default clock-divider is -> 80
    // default APB clk is available with the APB_CLK_FREQ constant
    let timer_conf = esp_idf_hal::timer::config::Config::new().auto_reload(true);
    let mut timer = esp_idf_hal::timer::TimerDriver::new(per.timer00, &timer_conf)?;

    // Calculate value needed for alarm in seconds
    // (APB_CLK_FREQ / DEVIDER ) * seconds = count
    // example every 200 us
    // ( 80*10^6 / 80 ) * 200 *10^(-6) = 200
    timer.set_alarm(200)?;

    // Saftey: make sure the task handle stays valid for longer than the subscribtion
    // is active
    unsafe {
        timer.subscribe(move || {
            let event_number = 42;
            esp_idf_hal::task::notify(main_task_handle, event_number);
        })?;
    }

    timer.enable_alarm(true)?;
    timer.enable(true)?;

    loop {
        // Notify approach
        // The benefit with this approach over checking a global static variable is
        // that the scheduler can hold the task, and resume when signaled
        // so no spinlock is needed
        let event_id = esp_idf_hal::task::wait_notification(None);

        // Note that the println functions are to slow for 200us
        // Even if we just send one charachter we can not go below 1ms per msg
        // so we are missing some events here - but if they are evaluated without
        // printing them the maintask will be fast enough no problem
        if let Some(event) = event_id {
            println!("got event with the number {event} from ISR");
        }
    }
}
