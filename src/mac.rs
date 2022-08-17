pub struct Mac(::core::marker::PhantomData<*const ()>);

impl Mac {
    /// # Safety
    ///
    /// Care should be taken not to instnatiate this Mac instance, if it is already instantiated and used elsewhere
    pub unsafe fn new() -> Self {
        Mac(::core::marker::PhantomData)
    }
}

unsafe impl Send for Mac {}
