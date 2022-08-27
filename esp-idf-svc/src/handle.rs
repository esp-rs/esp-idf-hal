pub trait RawHandle {
    type Handle;

    /// # Safety
    ///
    /// Care should be taken to use the returned ESP-IDF driver raw handle only while
    /// the driver is still alive, so as to avoid use-after-free errors.
    unsafe fn handle(&self) -> Self::Handle;
}
