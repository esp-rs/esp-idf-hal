use esp_idf_sys::mcpwm_operator_config_t__bindgen_ty_1;

use crate::gpio::OutputPin;

use super::{
    comparator::ComparatorConfig,
    generator::{ComparatorChannel, GeneratorConfig},
};

#[derive(Default)]
pub struct OperatorConfig<'d> {
    /// Configuration for Comparator X
    pub(crate) comparator_x: ComparatorConfig,

    /// Configuration for Comparator Y
    pub(crate) comparator_y: ComparatorConfig,

    /// Configuration for Generator A
    pub(crate) generator_a: Option<GeneratorConfig<'d>>,

    /// Configuration for Generator B
    pub(crate) generator_b: Option<GeneratorConfig<'d>>,
}

impl<'d> OperatorConfig<'d> {
    pub fn default<PA: OutputPin, PB: OutputPin>(pin_a: PA, pin_b: PB) -> OperatorConfig<'d> {
        OperatorConfig::empty()
            .cmp_x(Default::default())
            .cmp_y(Default::default())
            .gen_a(GeneratorConfig::active_high(pin_a, ComparatorChannel::CmpX))
            .gen_b(GeneratorConfig::active_high(pin_b, ComparatorChannel::CmpY))
    }

    pub fn empty() -> Self {
        let mut flags: mcpwm_operator_config_t__bindgen_ty_1 = Default::default();

        // TODO: What value makes most sense as default here?
        // TODO: When, how and who should set the flags?
        // Should that be done automagically, manually or does some specific setting cover all cases?
        // Flags for Operator
        flags.set_update_gen_action_on_tez(1);
        flags.set_update_gen_action_on_tep(1);
        flags.set_update_gen_action_on_sync(1);

        flags.set_update_dead_time_on_tez(1);
        flags.set_update_dead_time_on_tep(1);
        flags.set_update_dead_time_on_sync(1);

        OperatorConfig {
            comparator_x: Default::default(), // SOC_MCPWM_COMPARATORS_PER_OPERATOR is 2 for ESP32 and ESP32-S3
            comparator_y: Default::default(),

            generator_a: None, // One generator per pin, with a maximum of two generators per Operator
            generator_b: None,
        }
    }
}

impl<'d> OperatorConfig<'d> {
    fn cmp_x(mut self, config: ComparatorConfig) -> Self {
        self.comparator_x = config;
        self
    }

    fn cmp_y(mut self, config: ComparatorConfig) -> Self {
        self.comparator_y = config;
        self
    }
}

impl<'d> OperatorConfig<'d> {
    #[allow(clippy::type_complexity)]
    fn gen_a(mut self, config: GeneratorConfig<'d>) -> Self {
        self.generator_a = Some(config);
        self
    }
}

impl<'d> OperatorConfig<'d> {
    #[allow(clippy::type_complexity)]
    fn gen_b(mut self, config: GeneratorConfig<'d>) -> Self {
        self.generator_b = Some(config);
        self
    }
}
