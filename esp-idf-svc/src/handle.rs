//! Wrapper around ESP-IDF raw handles
pub trait RawHandle {
    type Handle;

    /// Care should be taken to use the returned ESP-IDF driver raw handle only while
    /// the driver is still alive, so as to avoid use-after-free errors.
    fn handle(&self) -> Self::Handle;
}

impl<R> RawHandle for &R
where
    R: RawHandle,
{
    type Handle = R::Handle;

    fn handle(&self) -> Self::Handle {
        (*self).handle()
    }
}

impl<R> RawHandle for &mut R
where
    R: RawHandle,
{
    type Handle = R::Handle;

    fn handle(&self) -> Self::Handle {
        (**self).handle()
    }
}
