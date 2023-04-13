pub(super) mod config {
        /// A/U-law compression/decompression configuration.
        #[cfg(esp_idf_soc_i2s_supports_pcm)]
        #[derive(Clone, Copy, Eq, PartialEq)]
        pub enum PcmCompress {
            /// Disable A/U-law compression/decompression.
            Disable,
    
            /// A-law decompression.
            ADecompress,
    
            /// A-law compression.
            ACompress,
    
            /// U-law decompression.
            UDecompress,
    
            /// U-law compression.
            UCompress,
        }
    
        #[cfg(esp_idf_soc_i2s_supports_pcm)]
        impl Default for PcmCompress {
            #[inline(always)]
            fn default() -> Self {
                Self::Disable
            }
        }
    
        #[cfg(esp_idf_soc_i2s_supports_pcm)]
        impl PcmCompress {
            /// Convert to the ESP-IDF SDK `i2s_pcm_compress_t` representation.
            #[inline(always)]
            #[allow(unused)] // TODO: remove when PCM is implemented.
            pub(super) fn as_sdk(&self) -> esp_idf_sys::i2s_pcm_compress_t {
                match self {
                    Self::Disable => 0,
                    Self::ADecompress => 1,
                    Self::ACompress => 2,
                    Self::UDecompress => 3,
                    Self::UCompress => 4,
                }
            }
        }    
}