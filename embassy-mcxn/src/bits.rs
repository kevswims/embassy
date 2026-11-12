//! Helpers for the PAC's anonymous field types.
//!
//! The PAC models flag fields as anonymous one-bit enums rather than `bool`, and
//! every register has its own enum type. Code that only knows the raw value (or
//! a `bool`) goes through these functions so it need not name those types.

/// A one-byte PAC field value (`bool` or a field enum) with the given raw bits.
///
/// `T` must be a `bool` or a field enum whose discriminants include `raw`.
pub const fn flag<T: Copy>(raw: u8) -> T {
    assert!(core::mem::size_of::<T>() == 1);
    // SAFETY: `T` is one byte, and the caller supplies a bit pattern that is
    // valid for it.
    unsafe { core::mem::transmute_copy(&raw) }
}

/// The raw bits of a one-byte PAC field value; the inverse of [`flag`].
pub fn raw<T: Copy>(value: T) -> u8 {
    assert!(core::mem::size_of::<T>() == 1);
    // SAFETY: `T` is one byte.
    unsafe { core::mem::transmute_copy(&value) }
}

/// A flag field value for `value`.
#[inline(always)]
pub fn from_bool<T: Copy>(value: bool) -> T {
    flag(value as u8)
}

/// Whether a flag field value is set.
#[inline(always)]
pub fn is_set<T: Copy>(value: T) -> bool {
    raw(value) != 0
}

/// A raw `u32` register value as the register's fieldset type.
pub const fn from_raw<T: Copy>(raw: u32) -> T {
    assert!(core::mem::size_of::<T>() == 4);
    // SAFETY: PAC fieldsets are `repr(transparent)` wrappers over `u32`.
    unsafe { core::mem::transmute_copy(&raw) }
}

/// An all-zero value of the fieldset type of `_reg`.
pub const fn zeroed_like<T: Copy, A: crate::pac::common::Access>(_reg: &crate::pac::common::Reg<T, A>) -> T {
    // SAFETY: PAC fieldsets are plain integer wrappers; zero is valid.
    unsafe { core::mem::zeroed() }
}
