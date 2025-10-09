use proc_macro2::TokenStream;
use quote::{quote, ToTokens};
use syn::spanned::Spanned;
use syn::{punctuated::Punctuated, Item, Token};

fn quote_link_section(name: &str, subsection: Option<impl quote::ToTokens>) -> TokenStream {
    // TODO: make sure #[unsafe()] is supported on the target rustc version?
    if let Some(subsection) = subsection {
        quote! {
            #[unsafe(link_section = ::core::concat!(#name, #subsection))]
        }
    } else {
        quote! {
            #[unsafe(link_section = #name)]
        }
    }
}

/// In the following code:
///
/// ```
/// #[link_section = ".dram1"]
/// static VARIABLE: &str = "Hello";
/// ```
///
/// only a pointer to the data + length will be stored in `.dram1`. The actual bytes
/// of the string will be stored in .flash.rodata
///
/// To do anything useful with the string, it will still have to access the flash...
/// The same problem applies to any static slices.
///
/// This function will rewrite the expression to explicitly define an owned buffer
/// that will be linked to .dram as well and the original expression will be a pointer
/// to that buffer.
///
/// To build the buffer, the type stored in the buffer must be [`Copy`], if not,
/// the macro will not be able to create the buffer.
///
/// There is no way to check whether a type is [`Copy`] in a proc-macro, which is why
/// this function has a `rewrite_any_expr` argument. If that is set to `true`,
/// the function will try to rewrite any expression, if not, it will only rewrite
/// string and byte string literals.
fn rewrite_static_expr(
    syn::ItemStatic {
        attrs,
        vis,
        ident,
        ty,
        mutability,
        expr,
        ..
    }: syn::ItemStatic,
    rewrite_any_expr: bool,
) -> syn::ItemStatic {
    let mut expr = (*expr).clone();

    let link_attr = quote_link_section(".dram1", Some(unique_section([&ident])));

    if let syn::Expr::Lit(syn::ExprLit { attrs, lit }) = &expr {
        let mut bytes: Option<Vec<u8>> = None;
        let mut from_buffer_expr: Option<syn::Expr> = None;

        match lit {
            syn::Lit::Str(lit_str) => {
                bytes = Some(lit_str.value().into_bytes());
                from_buffer_expr = Some(syn::parse_quote! {
                    unsafe { ::core::str::from_utf8_unchecked(&_BUFFER) }
                });
            }
            syn::Lit::ByteStr(lit_byte_str) => {
                bytes = Some(lit_byte_str.value());
                from_buffer_expr = Some(syn::parse_quote!(_BUFFER.as_slice()));
            }
            _ => {}
        }

        if let (Some(bytes), Some(from_buffer_expr)) = (bytes, from_buffer_expr) {
            let buffer_len = bytes.len();
            let buffer_expr: syn::ExprArray = syn::parse_quote!([#(#bytes),*]);

            expr = syn::parse_quote!(#(#attrs)* {
                #link_attr
                static _BUFFER: [u8; #buffer_len] = #buffer_expr;

                #from_buffer_expr
            });
        }
    }

    if rewrite_any_expr {
        // Check if it is an &[T] where T = inner_type
        if let syn::Type::Reference(syn::TypeReference { elem, .. }) = &*ty {
            if let syn::Type::Slice(syn::TypeSlice { elem, .. }) = &**elem {
                expr = syn::parse_quote_spanned!(elem.span() => {
                    const VALUE: #ty = #expr;
                    const SLICE_EXPR: &[#elem] = #expr;

                    #link_attr
                    static _BUFFER: [#elem; SLICE_EXPR.len()] = {
                        let mut buf: [::core::mem::MaybeUninit<#elem>; SLICE_EXPR.len()] =
                            unsafe { ::core::mem::MaybeUninit::uninit().assume_init() };

                        let mut i = 0;
                        while i < buf.len() {
                            buf[i] = ::core::mem::MaybeUninit::new(SLICE_EXPR[i]);
                            i += 1;
                        }

                        unsafe { ::core::mem::transmute::<_, [#elem; SLICE_EXPR.len()]>(buf) }
                    };

                    _BUFFER.as_slice()
                });
            }
        }
    }

    syn::parse_quote!(
        #(#attrs)*
        #link_attr
        #vis static #mutability #ident: #ty = #expr;
    )
}

struct Arguments {
    is_copy: bool,
}

impl syn::parse::Parse for Arguments {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let items = Punctuated::<syn::Meta, Token![,]>::parse_terminated(&input).unwrap();
        if items.len() == 1 {
            if let syn::Meta::Path(path) = &items[0] {
                if path.is_ident("copy") {
                    return Ok(Self { is_copy: true });
                }
            }
        } else if items.is_empty() {
            return Ok(Self { is_copy: false });
        }

        return Err(syn::Error::new(
            input.span(),
            "The only supported argument to the `#[ram]` attribute is `copy`",
        ));
    }
}

pub fn ram(args: TokenStream, input: TokenStream) -> syn::Result<TokenStream> {
    let args_span = args.span();
    let attr_args = syn::parse2::<Arguments>(args)?;

    let input_span = input.span();
    let item = syn::parse2::<Item>(input)?;
    match &item {
        Item::Static(item_static) => {
            Ok(rewrite_static_expr(item_static.clone(), attr_args.is_copy).into_token_stream())
        }
        Item::Fn(syn::ItemFn {
            sig: syn::Signature { ident, .. },
            ..
        }) => {
            // For now, only support empty attribute arguments
            if attr_args.is_copy {
                return Err(syn::Error::new(
                    args_span,
                    "Invalid argument to the `#[ram]` attribute on functions",
                ));
            }

            let link_attr = quote_link_section(".iram1", Some(unique_section([ident])));

            Ok(quote! {
                #link_attr
                #[inline(never)]
                #item
            })
        }
        _ => Err(syn::Error::new(
            input_span,
            "The `#[ram]` attribute can not be applied to this item",
        )),
    }
}

fn unique_section<T: quote::ToTokens>(idents: impl IntoIterator<Item = T>) -> syn::Expr {
    let separator = quote!("_",);
    let iter = idents.into_iter();
    syn::parse_quote!(::core::concat!(
        ".",
        #(::core::stringify!(#iter))(#separator)*,
        "_",
        ::core::line!(),
        "_",
        ::core::column!()
    ))
}
