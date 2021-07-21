#![allow(dead_code)]

/// This module is a manual translation of the following C file from current ESP-IDF master:
/// - https://github.com/espressif/esp-idf/blob/master/components/ulp/ulp_riscv/include/ulp_riscv/ulp_riscv_register_ops.h

use core::ptr::{read_volatile, write_volatile};

/*
 * When COCPU accesses the RTC register, it needs to convert the access address.
 * When COCPU accesses the RTC memory, dont need to convert the access address.
 */
 #[inline(always)]
 pub unsafe fn write_rtc_mem(addr: u32, val: i32) {
     write_volatile(addr as *mut i32, val);
 }

 #[inline(always)]
 pub unsafe fn read_rtc_mem(addr: u32) -> i32 {
     read_volatile(addr as *const i32)
 }

 /*
  * When COCPU accesses the RTC register, it needs to convert the access address.
  * When COCPU accesses the RTC memory, dont need to convert the access address.
  */
#[inline(always)]
pub const fn riscv_reg_conv(addr: u32) -> u32 {
    ((addr & 0xffff) << 3 & 0xe000) | addr & 0x1fff | 0x8000
}

#[inline(always)]
pub const fn ets_uncached_addr(addr: u32) -> u32 {
    riscv_reg_conv(addr)
}

#[inline(always)]
pub const fn bit(nr: u32) -> u32 {
    1u32 << nr
}

// Write value to register
#[inline(always)]
pub unsafe fn reg_write(r: u32, v: u32) {
    write_volatile(riscv_reg_conv(r) as *mut u32, v);
}

// Read value from register
#[inline(always)]
pub unsafe fn reg_read(r: u32) -> u32 {
    read_volatile(riscv_reg_conv(r) as *const u32)
}

// Get bit or get bits from register
#[inline(always)]
pub unsafe fn reg_get_bit(r: u32, b: u32) -> u32 {
    read_volatile(riscv_reg_conv(r) as *const u32) & b
}

// Get bit or get bits from register
#[inline(always)]
pub unsafe fn reg_set_bit(r: u32, b: u32) {
    let addr = riscv_reg_conv(r) as *mut u32;
    write_volatile(addr, read_volatile(addr) | b);
}

// Clear bit or clear bits of register
#[inline(always)]
pub unsafe fn reg_clr_bit(r: u32, b: u32) {
    let addr = riscv_reg_conv(r) as *mut u32;
    write_volatile(addr, read_volatile(addr) & (!b));
}

// Set bits of register controlled by mask
#[inline(always)]
pub unsafe fn reg_set_bits(r: u32, b: u32, m: u32) {
    let addr = riscv_reg_conv(r) as *mut u32;
    write_volatile(addr, read_volatile(addr) & (!m) | b & m);
}

// Get field from register, uses field _S & _V to determine mask
#[inline(always)]
pub unsafe fn reg_get_field(r: u32, f_s: u32, f_v: u32) -> u32 {
    (reg_read(r) >> f_s) & f_v
}

// Set field of a register from variable, uses field _S & _V to determine mask
#[inline(always)]
pub unsafe fn reg_set_field(r: u32, f_s: u32, f_v: u32, v: u32) {
    reg_write(r, (reg_read(r) & !(f_v << f_s)) | ((v & f_v) << f_s));
}

// Get field value from a variable, used when _f is not left shifted by _f##_S
#[inline(always)]
pub const fn value_get_field(r: u32, f: u32, f_s: u32) -> u32 {
    (r >> f_s) & f
}

// Get field value from a variable, used when _f is left shifted by _f##_S
#[inline(always)]
pub const fn value_get_field2(r: u32, f: u32, f_s: u32) -> u32 {
    (r & f) >> f_s
}

// Set field value to a variable, used when _f is not left shifted by _f##_S
#[inline(always)]
pub const fn value_set_field(r: u32, f: u32, f_s: u32, v: u32) -> u32 {
    r & !(f << f_s) | (v << f_s)
}

// Set field value to a variable, used when _f is left shifted by _f##_S
#[inline(always)]
pub const fn value_set_field2(r: u32, f: u32, f_s: u32, v: u32) -> u32 {
    r & !f | (v << f_s)
}

// Generate a value from a field value, used when _f is not left shifted by _f##_S
#[inline(always)]
pub const fn field_to_value(f: u32, f_s: u32, v: u32) -> u32 {
    (v & f) << f_s
}

// Generate a value from a field value, used when _f is left shifted by _f##_S
#[inline(always)]
pub const fn field_to_value2(f: u32, f_s: u32, v: u32) -> u32 {
    (v << f_s) & f
}

// Read value from register
#[inline(always)]
pub unsafe fn read_peri_reg(addr: u32) -> u32 {
    read_volatile(ets_uncached_addr(addr) as *const u32)
}

// Write value to register
#[inline(always)]
pub unsafe fn write_peri_reg(addr: u32, v: u32) {
    write_volatile(ets_uncached_addr(addr) as *mut u32, v);
}

// Clear bits of register controlled by mask
#[inline(always)]
pub unsafe fn clear_peri_reg_mask(addr: u32, mask: u32) {
    write_peri_reg(addr, read_peri_reg(addr) & !mask);
}

#[inline(always)]
pub unsafe fn set_peri_reg_mask(addr: u32, mask: u32) {
    write_peri_reg(addr, read_peri_reg(addr) | mask);
}

// Get bits of register controlled by mask
#[inline(always)]
pub unsafe fn get_peri_reg_mask(addr: u32, mask: u32) -> u32 {
    read_peri_reg(addr) & mask
}

// Get bits of register controlled by highest bit and lowest bit
#[inline(always)]
pub unsafe fn get_peri_reg_bits(addr: u32, bit_map: u32, shift: u8) -> u32 {
    (read_peri_reg(addr) & (bit_map << shift)) >> shift
}

// Set bits of register controlled by mask and shift
pub unsafe fn set_peri_reg_bits(addr: u32, bit_map: u32, value: u32, shift: u8) {
    write_peri_reg(addr, read_peri_reg(addr) & !(bit_map << shift) | ((value & bit_map) << shift));
}

// Get field of register
pub unsafe fn get_peri_reg_bits2(addr: u32, mask: u32, shift: u8) -> u32 {
    (read_peri_reg(addr) >> shift) & mask
}
