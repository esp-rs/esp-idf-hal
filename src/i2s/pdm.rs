pub(super) mod config {
    /// I2S pulse density modulation (PDM) downsampling mode.
    #[cfg(esp_idf_soc_i2s_supports_pdm_rx)]
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum PdmDownsample {
        /// Downsample 8 samples.
        Samples8,

        /// Downsample 16 samples.
        Samples16,

        /// Maximum downsample rate.
        Max,
    }

    #[cfg(esp_idf_soc_i2s_supports_pdm_rx)]
    impl Default for PdmDownsample {
        #[inline(always)]
        fn default() -> Self {
            Self::Samples8
        }
    }

    #[cfg(esp_idf_soc_i2s_supports_pdm_rx)]
    impl PdmDownsample {
        /// Convert to the ESP-IDF SDK `i2s_pdm_downsample_t` representation.
        #[inline(always)]
        #[allow(unused)] // TODO: remove when PDM is implemented.
        pub(super) fn as_sdk(&self) -> esp_idf_sys::i2s_pdm_dsr_t {
            match self {
                Self::Samples8 => 0,
                Self::Samples16 => 1,
                Self::Max => 2,
            }
        }
    }

    /// Pulse density modulation (PDM) transmit signal scaling mode.
    #[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum PdmSignalScale {
        /// Divide the PDM signal by 2.
        Div2,

        /// No scaling.
        None,

        /// Multiply the PDM signal by 2.
        Mul2,

        /// Multiply the PDM signal by 4.
        Mul4,
    }

    #[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
    impl Default for PdmSignalScale {
        #[inline(always)]
        fn default() -> Self {
            Self::None
        }
    }

    #[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
    impl PdmSignalScale {
        /// Convert to the ESP-IDF SDK `i2s_pdm_signal_scale_t` representation.
        #[inline(always)]
        #[allow(unused)] // TODO: remove when PDM is implemented.
        pub(crate) fn as_sdk(&self) -> esp_idf_sys::i2s_pdm_sig_scale_t {
            match self {
                Self::Div2 => 0,
                Self::None => 1,
                Self::Mul2 => 2,
                Self::Mul4 => 3,
            }
        }
    }
}
