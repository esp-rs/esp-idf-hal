use core::ptr;
use core::time::Duration;

use esp_idf_sys::*;

use crate::delay::TickType;
use crate::interrupt;

#[inline(always)]
#[link_section = ".iram1.interrupt_task_do_yield"]
pub fn do_yield() {
    if interrupt::active() {
        #[cfg(esp32c3)]
        unsafe {
            if let Some(yielder) = interrupt::get_isr_yielder() {
                yielder();
            } else {
                vPortYieldFromISR();
            }
        }

        #[cfg(not(esp32c3))]
        unsafe {
            if let Some(yielder) = interrupt::get_isr_yielder() {
                yielder();
            } else {
                #[cfg(esp_idf_version_major = "4")]
                vPortEvaluateYieldFromISR(0);

                #[cfg(esp_idf_version_major = "5")]
                _frxt_setup_switch();
            }
        }
    } else {
        unsafe {
            vPortYield();
        }
    }
}

#[inline(always)]
#[link_section = ".iram1.interrupt_task_current"]
pub fn current() -> Option<TaskHandle_t> {
    if interrupt::active() {
        None
    } else {
        Some(unsafe { xTaskGetCurrentTaskHandle() })
    }
}

pub fn wait_any_notification() {
    loop {
        if let Some(notification) = wait_notification(None) {
            if notification != 0 {
                break;
            }
        }
    }
}

pub fn wait_notification(duration: Option<Duration>) -> Option<u32> {
    let mut notification = 0_u32;

    #[cfg(esp_idf_version = "4.3")]
    let notified = unsafe {
        xTaskNotifyWait(
            0,
            u32::MAX,
            &mut notification as *mut _,
            TickType::from(duration).0,
        )
    } != 0;

    #[cfg(not(esp_idf_version = "4.3"))]
    let notified = unsafe {
        xTaskGenericNotifyWait(
            0,
            0,
            u32::MAX,
            &mut notification as *mut _,
            TickType::from(duration).0,
        )
    } != 0;

    if notified {
        Some(notification)
    } else {
        None
    }
}

/// # Safety
///
/// When calling this function care should be taken to pass a valid
/// FreeRTOS task handle. Moreover, the FreeRTOS task should be valid
/// when this function is being called.
pub unsafe fn notify(task: TaskHandle_t, notification: u32) -> bool {
    let notified = if interrupt::active() {
        let mut higher_prio_task_woken: BaseType_t = Default::default();

        #[cfg(esp_idf_version = "4.3")]
        let notified = xTaskGenericNotifyFromISR(
            task,
            notification,
            eNotifyAction_eSetBits,
            ptr::null_mut(),
            &mut higher_prio_task_woken as *mut _,
        );

        #[cfg(not(esp_idf_version = "4.3"))]
        let notified = xTaskGenericNotifyFromISR(
            task,
            0,
            notification,
            eNotifyAction_eSetBits,
            ptr::null_mut(),
            &mut higher_prio_task_woken as *mut _,
        );

        if higher_prio_task_woken != 0 {
            do_yield();
        }

        notified
    } else {
        #[cfg(esp_idf_version = "4.3")]
        let notified =
            xTaskGenericNotify(task, notification, eNotifyAction_eSetBits, ptr::null_mut());

        #[cfg(not(esp_idf_version = "4.3"))]
        let notified = xTaskGenericNotify(
            task,
            0,
            notification,
            eNotifyAction_eSetBits,
            ptr::null_mut(),
        );

        notified
    };

    notified != 0
}
