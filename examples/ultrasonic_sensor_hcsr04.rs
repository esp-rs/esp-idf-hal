use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use log::*;
use esp_idf_svc::systime::EspSystemTime;

fn main() {
    // Initialize logging and necessary peripherals
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("Hello, world!");

    let peripherals = Peripherals::take().unwrap();

    // Configure pins for trigger and echo
    let mut trigger_pin = PinDriver::output(peripherals.pins.gpio4).expect("Error configuring trigger pin");
    let echo_pin = PinDriver::input(peripherals.pins.gpio5).expect("Error configuring echo pin");


    loop {
        // Send a 10us pulse to the trigger pin to start the measurement
        trigger_pin.set_high().expect("Error: Unable to set trigger pin high");
        FreeRtos::delay_us(10);
        trigger_pin.set_low().expect("Error: Unable to set trigger pin low");

        // Wait for the echo pin to go high (start of pulse)
        while !echo_pin.is_high() {}

        // Measure the duration of the echo pulse (in microseconds)
        let start_time = EspSystemTime {}.now().as_micros();
        while echo_pin.is_high() {}
        let end_time = EspSystemTime {}.now().as_micros();

        // Calculate the duration of the echo pulse in microseconds
        let pulse_duration = end_time - start_time;

        // Calculate the distance based on the speed of sound (approximately 343 m/s)
        // Distance in centimeters: duration * speed_of_sound / 2 (since the signal goes to the object and back)
        let distance_cm = (pulse_duration as f32 * 0.0343) / 2.0;

        // Distance in inches: distance_cm / 2.54 (since 1 inch = 2.54 cm)
        let distance_inches = distance_cm / 2.54;

        // Print the measured distance in both centimeters and inches
        info!("Distance: {:.2} cm, {:.2} inches", distance_cm, distance_inches);

        // Wait for a brief moment before taking the next measurement
        FreeRtos::delay_ms(1000);
    }

}
