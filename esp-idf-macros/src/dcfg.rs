use proc_macro2::TokenStream;
use quote::{quote, quote_spanned};
use syn::spanned::Spanned;

pub fn dcfg(args: TokenStream, input: TokenStream) -> syn::Result<TokenStream> {
    // This will either emit #[cfg(...)] or #[cfg(...)] #[doc(cfg(...))] depending on whether
    // the "doc-cfg" feature is enabled.

    let attrs = {
        #[cfg(feature = "doc-cfg")]
        {
            quote_spanned!(args.span() => #[cfg(#args)] #[doc(cfg(#args))])
        }
        #[cfg(not(feature = "doc-cfg"))]
        {
            quote_spanned!(args.span() => #[cfg(#args)])
        }
    };

    Ok(quote! {
        #attrs
        #input
    })
}
