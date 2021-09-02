use core::mem;
use core::ptr;

extern crate alloc;
use alloc::vec;

use log::*;

use mutex_trait::*;

use embedded_svc::ota::{self, OtaUpdate};

use esp_idf_sys::*;

use crate::private::{common::*, cstr::*};

static mut TAKEN: EspMutex<bool> = EspMutex::new(false);

impl From<Newtype<&esp_app_desc_t>> for ota::FirmwareInfo {
    fn from(app_desc: Newtype<&esp_app_desc_t>) -> Self {
        let app_desc = app_desc.0;

        Self {
            version: from_cstr_ptr(&app_desc.version as *const _),
            signature: Some(app_desc.app_elf_sha256.into()),
            released: from_cstr_ptr(&app_desc.date as *const _)
                + &from_cstr_ptr(&app_desc.time as *const _),
            description: from_cstr_ptr(&app_desc.project_name as *const _),
        }
    }
}

pub struct EspFirmwareInfoLoader(vec::Vec<u8>);

impl EspFirmwareInfoLoader {
    pub fn new() -> Self {
        Self(vec::Vec::new())
    }
}

impl Default for EspFirmwareInfoLoader {
    fn default() -> Self {
        Self::new()
    }
}

impl ota::FirmwareInfoLoader for EspFirmwareInfoLoader {
    type Error = EspError;

    fn load(&mut self, buf: &[u8]) -> Result<ota::LoadResult, Self::Error> {
        if !self.is_loaded() {
            self.0.extend_from_slice(buf);
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

            Ok(ota::FirmwareInfo::from(Newtype(app_desc)))
        } else {
            Err(EspError::from(ESP_ERR_INVALID_SIZE as _).unwrap())
        }
    }
}

pub struct EspSlot(esp_partition_t);

impl ota::Slot for EspSlot {
    type Error = EspError;

    fn get_label(&self) -> Result<String, Self::Error> {
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

            Some(ota::FirmwareInfo::from(Newtype(&app_desc)))
        })
    }
}

pub struct Read;

pub struct Update(esp_ota_handle_t);

#[derive(Debug)]
pub struct EspOta<MODE>(MODE);

impl EspOta<Read> {
    pub fn new() -> Result<Self, EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                if *taken {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    *taken = true;
                    Ok(Self(Read))
                }
            })
        }
    }
}

impl<MODE> Drop for EspOta<MODE> {
    fn drop(&mut self) {
        unsafe {
            TAKEN.lock(|taken| {
                *taken = false;
            });
        }

        info!("Dropped");
    }
}

impl ota::Ota for EspOta<Read> {
    type Slot = EspSlot;
    type OtaUpdate = EspOta<Update>;
    type Error = EspError;

    fn get_boot_slot<'a>(&'a self) -> Result<Self::Slot, Self::Error>
    where
        Self::Slot: 'a,
    {
        Ok(EspSlot(unsafe {
            *esp_ota_get_boot_partition().as_ref().unwrap()
        }))
    }

    fn get_running_slot<'a>(&'a self) -> Result<Self::Slot, Self::Error>
    where
        Self::Slot: 'a,
    {
        Ok(EspSlot(unsafe {
            *esp_ota_get_boot_partition().as_ref().unwrap()
        }))
    }

    fn get_update_slot<'a>(&'a self) -> Result<Self::Slot, Self::Error>
    where
        Self::Slot: 'a,
    {
        Ok(EspSlot(unsafe {
            *esp_ota_get_next_update_partition(ptr::null())
                .as_ref()
                .unwrap()
        }))
    }

    fn factory_reset(self) -> Self::Error {
        todo!()
    }

    fn initiate_update(self) -> Result<Self::OtaUpdate, Self::Error> {
        let partition = unsafe { esp_ota_get_next_update_partition(ptr::null()) };

        let mut out_handle: esp_ota_handle_t = Default::default();

        esp!(unsafe { esp_ota_begin(partition, OTA_SIZE_UNKNOWN, &mut out_handle as *mut _) })?;

        Ok(EspOta(Update(out_handle)))
    }

    fn mark_running_slot_valid(&mut self) -> Result<(), Self::Error> {
        esp!(unsafe { esp_ota_mark_app_valid_cancel_rollback() })
    }

    fn mark_running_slot_invalid_and_reboot(&mut self) -> Self::Error {
        if let Err(err) = esp!(unsafe { esp_ota_mark_app_invalid_rollback_and_reboot() }) {
            err
        } else {
            unreachable!()
        }
    }
}

impl ota::OtaUpdate for EspOta<Update> {
    type Ota = EspOta<Read>;
    type Error = EspError;

    fn write_buf(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        esp!(unsafe { esp_ota_write(self.0 .0, buf as *const _ as *const _, buf.len() as _) })
    }

    fn complete(self) -> Result<Self::Ota, Self::Error> {
        esp!(unsafe { esp_ota_end(self.0 .0) })?;

        // TODO: esp_ota_set_boot_partition

        Ok(EspOta(Read))
    }

    fn abort(self) -> Result<Self::Ota, Self::Error> {
        esp!(unsafe { esp_ota_abort(self.0 .0) })?;

        Ok(EspOta(Read))
    }
}

#[cfg(feature = "std")]
impl std::io::Write for EspOta<Update> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, std::io::Error> {
        self.write_buf(buf)
            .map(|_| buf.len())
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))
    }

    fn flush(&mut self) -> Result<(), std::io::Error> {
        Ok(())
    }
}
