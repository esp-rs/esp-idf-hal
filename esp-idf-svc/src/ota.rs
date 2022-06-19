use core::cmp::min;
use core::fmt::Write;
use core::mem;
use core::ptr;

use ::log::*;

use embedded_svc::io;
use embedded_svc::ota;

use esp_idf_hal::mutex;

use esp_idf_sys::*;

use crate::errors::EspIOError;
use crate::private::{common::*, cstr::*};

static TAKEN: mutex::Mutex<bool> = mutex::Mutex::new(false);

impl From<Newtype<&esp_app_desc_t>> for ota::FirmwareInfo {
    fn from(app_desc: Newtype<&esp_app_desc_t>) -> Self {
        let app_desc = app_desc.0;

        let mut result = Self {
            version: from_cstr_ptr(&app_desc.version as *const _).into(),
            signature: Some(heapless::Vec::from_slice(&app_desc.app_elf_sha256).unwrap()),
            released: "".into(),
            description: Some(from_cstr_ptr(&app_desc.project_name as *const _).into()),
            download_id: None,
        };

        write!(
            &mut result.released,
            "{}{}",
            from_cstr_ptr(&app_desc.date as *const _),
            from_cstr_ptr(&app_desc.time as *const _)
        )
        .unwrap();

        result
    }
}

pub struct EspFirmwareInfoLoader(heapless::Vec<u8, 512>);

impl EspFirmwareInfoLoader {
    pub fn new() -> Self {
        Self(heapless::Vec::new())
    }
}

impl Default for EspFirmwareInfoLoader {
    fn default() -> Self {
        Self::new()
    }
}

impl io::Io for EspFirmwareInfoLoader {
    type Error = EspIOError;
}

impl ota::FirmwareInfoLoader for EspFirmwareInfoLoader {
    fn load(&mut self, buf: &[u8]) -> Result<ota::LoadResult, Self::Error> {
        if !self.is_loaded() {
            let remaining = self.0.capacity() - self.0.len();
            if remaining > 0 {
                self.0
                    .extend_from_slice(&buf[..min(buf.len(), remaining)])
                    .unwrap();
            }
        }

        Ok(if self.is_loaded() {
            ota::LoadResult::Loaded
        } else {
            ota::LoadResult::LoadMore
        })
    }

    fn is_loaded(&self) -> bool {
        self.0.len()
            >= mem::size_of::<esp_image_header_t>()
                + mem::size_of::<esp_image_segment_header_t>()
                + mem::size_of::<esp_app_desc_t>()
    }

    fn get_info(&self) -> Result<ota::FirmwareInfo, Self::Error> {
        if self.is_loaded() {
            let app_desc_slice = &self.0[0..mem::size_of::<esp_image_header_t>()
                + mem::size_of::<esp_image_segment_header_t>()];

            let app_desc = unsafe {
                (app_desc_slice.as_ptr() as *const esp_app_desc_t)
                    .as_ref()
                    .unwrap()
            };

            Ok(Newtype(app_desc).into())
        } else {
            Err(EspError::from(ESP_ERR_INVALID_SIZE as _).unwrap().into())
        }
    }
}

pub struct EspSlot(esp_partition_t);

impl io::Io for EspSlot {
    type Error = EspIOError;
}

impl ota::OtaSlot for EspSlot {
    fn get_label(&self) -> Result<&str, Self::Error> {
        Ok(from_cstr_ptr(&self.0.label as *const _ as *const _))
    }

    fn get_state(&self) -> Result<ota::SlotState, Self::Error> {
        let mut state: esp_ota_img_states_t = Default::default();

        let err = unsafe { esp_ota_get_state_partition(&self.0 as *const _, &mut state as *mut _) };

        Ok(if err == ESP_ERR_NOT_FOUND as i32 {
            ota::SlotState::Unknown
        } else {
            esp!(err)?;

            #[allow(non_upper_case_globals)]
            match state {
                esp_ota_img_states_t_ESP_OTA_IMG_NEW
                | esp_ota_img_states_t_ESP_OTA_IMG_PENDING_VERIFY => ota::SlotState::Unverified,
                esp_ota_img_states_t_ESP_OTA_IMG_VALID => ota::SlotState::Valid,
                esp_ota_img_states_t_ESP_OTA_IMG_INVALID
                | esp_ota_img_states_t_ESP_OTA_IMG_ABORTED => ota::SlotState::Invalid,
                esp_ota_img_states_t_ESP_OTA_IMG_UNDEFINED => ota::SlotState::Unknown,
                _ => ota::SlotState::Unknown,
            }
        })
    }

    fn get_firmware_info(&self) -> Result<Option<ota::FirmwareInfo>, Self::Error> {
        let mut app_desc: esp_app_desc_t = Default::default();

        let err = unsafe { esp_ota_get_partition_description(&self.0 as *const _, &mut app_desc) };

        Ok(if err == ESP_ERR_NOT_FOUND as i32 {
            None
        } else {
            esp!(err)?;

            Some(Newtype(&app_desc).into())
        })
    }
}

pub struct Read;

pub struct Update {
    partition: *const esp_partition_t,
    handle: esp_ota_handle_t,
}

#[derive(Debug)]
pub struct EspOta<MODE>(MODE);

impl EspOta<Read> {
    pub fn new() -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE as i32)?;
        }

        *taken = true;
        Ok(Self(Read))
    }

    fn get_factory_partition(&self) -> Result<*const esp_partition_t, EspError> {
        let partition_iterator = unsafe {
            esp_partition_find(
                esp_partition_type_t_ESP_PARTITION_TYPE_APP,
                esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_FACTORY,
                b"factory\0" as *const _ as *const _,
            )
        };

        if partition_iterator.is_null() {
            esp!(ESP_ERR_NOT_SUPPORTED)?;
        }

        let partition = unsafe { esp_partition_get(partition_iterator) };

        unsafe { esp_partition_iterator_release(partition_iterator) };

        Ok(partition)
    }
}

impl<MODE> Drop for EspOta<MODE> {
    fn drop(&mut self) {
        *TAKEN.lock() = false;

        info!("Dropped");
    }
}

impl<P> io::Io for EspOta<P> {
    type Error = EspIOError;
}

impl ota::Ota for EspOta<Read> {
    type Slot<'a> = EspSlot;
    type Update<'a> = EspOta<Update>;

    fn get_boot_slot(&self) -> Result<Self::Slot<'_>, Self::Error> {
        Ok(EspSlot(unsafe {
            *esp_ota_get_boot_partition().as_ref().unwrap()
        }))
    }

    fn get_running_slot(&self) -> Result<Self::Slot<'_>, Self::Error> {
        Ok(EspSlot(unsafe {
            *esp_ota_get_boot_partition().as_ref().unwrap()
        }))
    }

    fn get_update_slot(&self) -> Result<Self::Slot<'_>, Self::Error> {
        Ok(EspSlot(unsafe {
            *esp_ota_get_next_update_partition(ptr::null())
                .as_ref()
                .unwrap()
        }))
    }

    fn is_factory_reset_supported(&self) -> Result<bool, Self::Error> {
        Ok(self
            .get_factory_partition()
            .map(|factory| !factory.is_null())?)
    }

    fn factory_reset(&mut self) -> Result<(), Self::Error> {
        let factory = self.get_factory_partition()?;

        esp!(unsafe { esp_ota_set_boot_partition(factory) })?;

        Ok(())
    }

    fn initiate_update(&mut self) -> Result<Self::Update<'_>, Self::Error> {
        let partition = unsafe { esp_ota_get_next_update_partition(ptr::null()) };

        let mut handle: esp_ota_handle_t = Default::default();

        esp!(unsafe { esp_ota_begin(partition, OTA_SIZE_UNKNOWN, &mut handle as *mut _) })?;

        Ok(EspOta(Update { partition, handle }))
    }

    fn mark_running_slot_valid(&mut self) -> Result<(), Self::Error> {
        Ok(esp!(unsafe { esp_ota_mark_app_valid_cancel_rollback() })?)
    }

    fn mark_running_slot_invalid_and_reboot(&mut self) -> Self::Error {
        if let Err(err) = esp!(unsafe { esp_ota_mark_app_invalid_rollback_and_reboot() }) {
            err.into()
        } else {
            unreachable!()
        }
    }
}

impl ota::OtaUpdate for EspOta<Update> {
    fn complete(self) -> Result<(), Self::Error> {
        esp!(unsafe { esp_ota_end(self.0.handle) })?;
        esp!(unsafe { esp_ota_set_boot_partition(self.0.partition) })?;

        Ok(())
    }

    fn abort(self) -> Result<(), Self::Error> {
        esp!(unsafe { esp_ota_abort(self.0.handle) })?;

        Ok(())
    }
}

impl io::Write for EspOta<Update> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        esp!(unsafe { esp_ota_write(self.0.handle, buf.as_ptr() as _, buf.len() as _) })?;

        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
