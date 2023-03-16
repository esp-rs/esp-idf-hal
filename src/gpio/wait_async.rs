use core::{
    cell::RefCell,
    future::Future,
    sync::atomic::{AtomicBool, Ordering},
    task::{Poll, Waker},
};

extern crate alloc;
use crate::gpio::{InputMode, InterruptType, Pin, PinDriver};
use alloc::sync::Arc;
use esp_idf_sys::{esp_nofail, gpio_intr_disable, EspError};

impl<T: Pin, MODE: InputMode> embedded_hal_async::digital::Wait for PinDriver<'_, T, MODE> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Ok(InputFuture::new(self, InterruptType::HighLevel)?.await)
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Ok(InputFuture::new(self, InterruptType::LowLevel)?.await)
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Ok(InputFuture::new(self, InterruptType::PosEdge)?.await)
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Ok(InputFuture::new(self, InterruptType::NegEdge)?.await)
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Ok(InputFuture::new(self, InterruptType::AnyEdge)?.await)
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
    // We don't need a mutex because the only possible data race is in the ISR
    // and we can just disable interrupts.
    optional_waker: Arc<RefCell<Option<Waker>>>,
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
            optional_waker: Arc::new(RefCell::new(None)),
        };
        unsafe {
            let processed_interrupt = res.processed_interrupt.clone();
            let cloned_waker = res.optional_waker.clone();
            let driver_pin = res.driver.pin();
            let callback = move || {
                // Mark that we processed the interrupt
                processed_interrupt.store(true, Ordering::Relaxed);
                // If we were not ready to be polled, we will have a waker to use

                // SAFETY - interrupts must be disabled while setting this
                // option to Some. While it is likely to be a thread safe
                // operation in practice anyway:
                //
                // a. there is no guarantee
                //
                // b. the compiler takes advantage of the fact that there is no
                // need for thread safety.
                //
                // c. if we don't do it "right", a&b guarantee it will
                // eventually break in some weird way some future person will be
                // forced to track down.
                if let Some(waker) = cloned_waker.borrow().as_ref() {
                    waker.wake_by_ref();
                }
                // Disable interrupts on thet way out.
                esp_nofail!(gpio_intr_disable(driver_pin));
            };

            res.driver.subscribe(callback)?;
            res.driver.enable_interrupt()?;
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
            return Poll::Ready(());
        } else {
            // SAFETY: to ensure thread safety of setting the waker vs using the
            // waker, we disable interrupts on the pin while setting the waker.
            // This guarantees the interrupt handler will not try to read the
            // option value while we are setting it.  It will either see it
            // right before we disable interrupts, if a context switch happens,
            // or right after we are done and re-enable interrupts.
            res.driver.disable_interrupt().unwrap();
            let mut mut_option = res.optional_waker.as_ref().borrow_mut();
            *mut_option = Some(cx.waker().clone());
            res.driver.enable_interrupt().unwrap();
            return Poll::Pending;
        }
    }
}
