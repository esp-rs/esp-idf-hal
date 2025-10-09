use proc_macro::TokenStream;

mod dcfg;
mod ram;

/// This attribute places the annotated function or static variable into the internal RAM
/// of the ESP32 chip.
///
/// # Possible issues with functions
///
/// If a function is placed into RAM, the literals are not automatically placed into RAM as well:
///
/// ```rust,ignore
/// #[ram]
/// fn gpio_isr_handler() -> usize {
///     let s = "I am string still stored in flash";
/// }
/// ```
///
/// To store the literal in flash, one could do
/// ```rust,ignore
/// #[ram]
/// fn gpio_isr_handler() -> usize {
///     #[ram]
///     static _S: &str = "I am string still stored in flash";
///     let s = _S;
/// }
/// ```
///
/// # Possible issues with statics that reference data
///
/// If the attribute is applied to a static variable that references some data, for example
/// `#[ram] static MESSAGE: &str = "Error, invalid argument";` or `#[ram] static BUFFER: &[u8] = &[0]`
/// the referenced data might not be placed into RAM.
///
/// For byte string and string literals, the attribute will ensure that the referenced data is placed in
/// RAM, but for arbitrary slices or references it is unable to do so.
///
/// The first option is to declare an owned static array that is then referenced by the static variable,
/// applying the ram attribute to both:
///
/// ```rust,ignore
/// #[ram]
/// static BUFFER_DATA: [u8; 3] = [0, 1, 2];
/// #[ram]
/// static BUFFER: &[u8] = &BUFFER_DATA;
/// ```
///
/// or like this:
///
/// ```rust,ignore
/// #[ram]
/// static BUFFER: &[u8] = {
///     #[ram]
///     static DATA: [u8; 3] = [0, 1, 2];
///     DATA.as_slice()
/// };
/// ```
///
/// If the slice value is [`Copy`], the attribute can do this automatically:
/// ```rust,ignore
/// #[ram(copy)]
/// static BUFFER: [u8; 3] = &[0, 1, 2];
/// ```
#[proc_macro_attribute]
pub fn ram(args: TokenStream, input: TokenStream) -> TokenStream {
    ram::ram(args.into(), input.into())
        .unwrap_or_else(|err| err.to_compile_error().into())
        .into()
}

/// This attribute forwards its arguments to `cfg`, and if the `doc-cfg` feature of this
/// crate is enabled, it will emit a `doc(cfg(...))` attribute as well.
///
/// # Example
///
/// For rustdoc to document that a piece of code is only available when a specific condition is met,
/// one would have to write (because it is nightly-only):
///
/// ```rust,ignore
/// pub enum ZeroCrossMode {
//      PositionZero,
//      NegativeZero,
//      NegativePosition,
//      PositiveNegative,
//      #[cfg(esp_idf_version_at_least_5_4_0)]
//      #[cfg_attr(feature = "nightly", doc(cfg(esp_idf_version_at_least_5_4_0)))]
//      Invalid,
//  }
/// ```
///
/// with this attribute, one can shorten it to:
///
/// ```rust,ignore
/// pub enum ZeroCrossMode {
//      PositionZero,
//      NegativeZero,
//      NegativePosition,
//      PositiveNegative,
//      #[dcfg(esp_idf_version_at_least_5_4_0)]
//      Invalid,
//  }
/// ```
///
/// The macro will then expand to the following if the `doc-cfg` feature is enabled:
///
/// ```rust,ignore
/// pub enum ZeroCrossMode {
///     PositionZero,
///     NegativeZero,
///     NegativePosition,
///     PositiveNegative,
///     #[cfg(esp_idf_version_at_least_5_4_0)]
///     #[doc(cfg(esp_idf_version_at_least_5_4_0))]
///     Invalid,
/// }
/// ```
///
/// and to the following if the `doc-cfg` feature is not enabled:
///
/// ```rust,ignore
/// pub enum ZeroCrossMode {
///     PositionZero,
///     NegativeZero,
///     NegativePosition,
///     PositiveNegative,
///     #[cfg(esp_idf_version_at_least_5_4_0)]
///     Invalid,
/// }
/// ```
#[proc_macro_attribute]
pub fn dcfg(args: TokenStream, input: TokenStream) -> TokenStream {
    dcfg::dcfg(args.into(), input.into())
        .unwrap_or_else(|err| err.to_compile_error().into())
        .into()
}
