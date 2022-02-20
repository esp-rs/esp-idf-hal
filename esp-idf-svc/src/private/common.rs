use core::cell::UnsafeCell;

pub struct Newtype<T>(pub T);

pub struct UnsafeCellSendSync<T>(pub UnsafeCell<T>);

unsafe impl<T> Send for UnsafeCellSendSync<T> {}
unsafe impl<T> Sync for UnsafeCellSendSync<T> {}
