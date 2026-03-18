//! Queue set example
//!
//! Demonstrates how to use [`QueueSet2`] to block on two queues of different
//! item types simultaneously.
//!
//! Two producer threads send items to separate queues:
//!   - Thread A sends `u32` sensor readings every second.
//!   - Thread B sends `f32` temperature values every 1.5 seconds.
//!
//! The main task uses `QueueSet2` to receive from whichever queue has an item
//! ready first, without polling or dedicated per-queue receive loops.

use std::sync::Arc;
use std::time::Duration;

use esp_idf_hal::task::queue::{Queue, QueueSet2, QueueSet2Selected};

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    println!("Queue set example started");

    // Create two queues with a capacity of 4 items each.
    // Wrap them in Arc so they can be shared with producer threads.
    let q_sensor: Arc<Queue<u32>> = Arc::new(Queue::new(4));
    let q_temp: Arc<Queue<f32>> = Arc::new(Queue::new(4));

    // Clone Arc handles for the producer threads.
    let q_sensor_tx = Arc::clone(&q_sensor);
    let q_temp_tx = Arc::clone(&q_temp);

    // Producer A: sends u32 sensor readings.
    std::thread::Builder::new()
        .stack_size(4096)
        .spawn(move || {
            for reading in 0u32.. {
                println!("[producer A] sending sensor reading: {reading}");
                q_sensor_tx
                    .send_back(reading, esp_idf_hal::delay::BLOCK)
                    .unwrap();
                std::thread::sleep(Duration::from_millis(1000));
            }
        })?;

    // Producer B: sends f32 temperature values.
    std::thread::Builder::new()
        .stack_size(4096)
        .spawn(move || {
            let mut temp = 20.0f32;
            loop {
                println!("[producer B] sending temperature: {temp:.1}");
                q_temp_tx
                    .send_back(temp, esp_idf_hal::delay::BLOCK)
                    .unwrap();
                temp += 0.5;
                std::thread::sleep(Duration::from_millis(1500));
            }
        })?;

    // Build the queue set.  Both queues are still empty here, which is
    // required by FreeRTOS before adding them to a set.
    let qs = QueueSet2::new(&*q_sensor, &*q_temp)?;

    println!("Listening on queue set …");

    loop {
        // Block until either queue has an item; portMAX_DELAY = wait forever.
        match qs.select_from_set(esp_idf_hal::delay::BLOCK) {
            Some(QueueSet2Selected::First(v)) => {
                println!("[consumer] sensor reading from q_sensor: {v}");
            }
            Some(QueueSet2Selected::Second(v)) => {
                println!("[consumer] temperature from q_temp: {v:.1}");
            }
            None => {
                // Timeout (won't happen with BLOCK, but handle it anyway).
                println!("[consumer] select_from_set timed out");
            }
        }
    }
}
