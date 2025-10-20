use proc_macro::TokenStream;

mod dcfg;
mod ram;

/// This attribute places the annotated function or static variable into the internal RAM
/// of the ESP32 chip.
///
/// This macro is not a magic bullet, there are several caveats that have to be considered
/// when using it.
///
/// If a function is placed into RAM, the literals are not automatically placed into RAM as well:
///
/// ```
/// #[ram]
/// fn gpio_isr_handler() -> usize {
///     let s = "I am string still stored in flash";
/// }
/// ```
///
/// To store the literal in flash, one could do
/// ```
/// #[ram]
/// fn gpio_isr_handler() -> usize {
///     #[ram]
///     static _S: &str = "I am string still stored in flash";
///     let s = _S;
/// }
/// ```
/// The macro does not can not place called functions into RAM automatically. This **must**
/// be done manually by annotationing the called functions with `#[ram]` where they are declared.
/// The same applies to any functions that are transitively called.
///
/// If the attribute is applied to a static variable that references some data, for example
/// `#[ram] static DATA: &[u8] = &OTHER_VARIABLE;` the referenced data might not be placed into RAM.
///
/// For byte string (`b"..."`) and string literals (`"..."`), the attribute will ensure that
/// the referenced data is placed in RAM, but for other expressions the caller has to ensure that.
///
/// If the expressions is referencing a slice, and the value is [`Copy`],
/// the variable can be annotated with `#[ram(copy)]` like this:
/// ```
/// #[ram(copy)]
/// static BUFFER: [u8; 3] = &[0, 1, 2];
/// ```
///
/// This will instruct to generate a constant expression that will ensure that the data is placed into RAM.
///
/// If that is not an option, one can also write:
/// ```
/// #[ram]
/// static BUFFER_DATA: [u8; 3] = [0, 1, 2];
/// #[ram]
/// static BUFFER: &[u8] = &BUFFER_DATA;
/// ```
///
/// or like this:
///
/// ```
/// #[ram]
/// static BUFFER: &[u8] = {
///     #[ram]
///     static DATA: [u8; 3] = [0, 1, 2];
///     DATA.as_slice()
/// };
/// ```
///
///
/// <div class="warning">
///
/// If the code is supposed to run while the flash is disabled, it is **strongly** recommended to
/// inspect the generated binary to ensure that the entire code and data is placed into RAM.
/// It might be necessary to implement that code in C or manually call the esp-idf functions
/// through `esp-idf-sys` to ensure that no flash functions are called.
///
/// For more information, refer to <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_flash/spi_flash_concurrency.html#iram-safe-interrupt-handlers>
///
/// </div>
#[proc_macro_attribute]
pub fn ram(args: TokenStream, input: TokenStream) -> TokenStream {
    ram::ram(args.into(), input.into())
        .unwrap_or_else(|err| err.to_compile_error())
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
/// ```
/// pub enum ZeroCrossMode {
///     PositionZero,
///     NegativeZero,
///     NegativePosition,
///     PositiveNegative,
///     #[cfg(esp_idf_version_at_least_5_4_0)]
///     #[cfg_attr(feature = "nightly", doc(cfg(esp_idf_version_at_least_5_4_0)))]
///     Invalid,
///  }
/// ```
///
/// with this attribute, one can shorten it to:
///
/// ```
/// pub enum ZeroCrossMode {
///     PositionZero,
///     NegativeZero,
///     NegativePosition,
///     PositiveNegative,
///     #[dcfg(esp_idf_version_at_least_5_4_0)]
///     Invalid,
/// }
/// ```
///
/// The macro will then expand to the following if the `doc-cfg` feature is enabled:
///
/// ```
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
/// ```
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
        .unwrap_or_else(|err| err.to_compile_error())
        .into()
}
