use core::{
    future::Future,
    sync::atomic::{AtomicBool, Ordering},
    task::Poll,
};
extern crate alloc;
use crate::gpio::{InputMode, InterruptType, Pin, PinDriver};
use alloc::sync::Arc;
use atomic_waker::AtomicWaker;
use esp_idf_sys::{esp_nofail, gpio_intr_disable, EspError};

impl<T: Pin, MODE: InputMode> embedded_hal_async::digital::Wait for PinDriver<'_, T, MODE> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        InputFuture::new(self, InterruptType::HighLevel)?.await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        InputFuture::new(self, InterruptType::LowLevel)?.await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        InputFuture::new(self, InterruptType::PosEdge)?.await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        InputFuture::new(self, InterruptType::NegEdge)?.await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        InputFuture::new(self, InterruptType::AnyEdge)?.await;
        Ok(())
    }
}

pub(crate) struct InputFuture<'driver_ref, 'driver_struct, T: Pin, MODE: InputMode> {
    // Unfortunately, the Wait trait uses functions that are given a mutable
    // reference to the pin driver with no explicit lifetime parameter.  Since
    // the reference has a different lifetime than the struct, we must make this
    // explicit, or else the borrow checker will believe the reference must live
    // forever.
    driver: &'driver_ref mut PinDriver<'driver_struct, T, MODE>,
    processed_interrupt: Arc<AtomicBool>,
    atomic_waker: Arc<AtomicWaker>,
}

impl<'driver_ref, 'driver_struct, T: Pin, MODE: InputMode>
    InputFuture<'driver_ref, 'driver_struct, T, MODE>
{
    pub(crate) fn new(
        driver: &'driver_ref mut PinDriver<'driver_struct, T, MODE>,
        interrupt_type: InterruptType,
    ) -> Result<Self, EspError> {
        driver.unsubscribe()?;
        driver.disable_interrupt()?;
        driver.set_interrupt_type(interrupt_type)?;

        let res = Self {
            driver,
            processed_interrupt: Arc::new(AtomicBool::new(false)),
            atomic_waker: Arc::new(AtomicWaker::new()),
        };
        unsafe {
            let processed_interrupt = res.processed_interrupt.clone();
            let cloned_waker = res.atomic_waker.clone();
            let driver_pin = res.driver.pin();
            let callback = move || {
                // Mark that we processed the interrupt
                processed_interrupt.store(true, Ordering::Relaxed);
                // If we were not ready to be polled, we will have a waker to use
                cloned_waker.wake();
                // Disable interrupts on thet way out.
                esp_nofail!(gpio_intr_disable(driver_pin));
            };

            res.driver.subscribe(callback)?;
        };
        Ok(res)
    }
}
impl<T: Pin, MODE: InputMode> Drop for InputFuture<'_, '_, T, MODE> {
    fn drop(&mut self) {
        self.driver.unsubscribe().unwrap();
    }
}

impl<T: Pin, MODE: InputMode> Unpin for InputFuture<'_, '_, T, MODE> {}
impl<T: Pin, MODE: InputMode> Future for InputFuture<'_, '_, T, MODE> {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let res = self.get_mut();

        // See if we processed the interrupt.
        // We could also look at the CPU to see if the interrupt was disabled, but
        // this is easier ATM.
        if res.processed_interrupt.load(Ordering::Relaxed) {
            Poll::Ready(())
        } else {
            res.atomic_waker.register(cx.waker());
            return Poll::Pending;
        }
    }
}
