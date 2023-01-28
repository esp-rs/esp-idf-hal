use esp_idf_sys::mcpwm_operator_config_t__bindgen_ty_1;

use crate::gpio::OutputPin;

use super::{
    comparator::ComparatorConfig,
    generator::{GenA, GenB, GeneratorConfig, NoGenCfg, OptionalGenCfg},
};

#[derive(Default)]
pub struct OperatorConfig<GENA, GENB> {
    // TODO: When, how and who should set the flags?
    // Should that be done automagically, manually or does some specific setting cover all cases?
    /// Flags for Operator
    pub(crate) flags: mcpwm_operator_config_t__bindgen_ty_1,

    /// Configuration for Comparator X
    pub(crate) comparator_x: ComparatorConfig,

    /// Configuration for Comparator Y
    pub(crate) comparator_y: ComparatorConfig,

    /// Configuration for Generator A
    pub(crate) generator_a: GENA,

    /// Configuration for Generator B
    pub(crate) generator_b: GENB,
}

impl OperatorConfig<NoGenCfg, NoGenCfg> {
    pub fn default<PA: OutputPin, PB: OutputPin>(
        pin_a: PA,
        pin_b: PB,
    ) -> OperatorConfig<GeneratorConfig<GenA, PA>, GeneratorConfig<GenB, PB>> {
        OperatorConfig::empty()
            .cmp_x(Default::default())
            .cmp_y(Default::default())
            .gen_a(GeneratorConfig::active_high(pin_a))
            .gen_b(GeneratorConfig::active_high(pin_b))
    }

    pub fn empty() -> Self {
        let mut flags: mcpwm_operator_config_t__bindgen_ty_1 = Default::default();

        // TODO: What value makes most sense as default here?
        flags.set_update_gen_action_on_tez(1);
        flags.set_update_gen_action_on_tep(1);
        flags.set_update_gen_action_on_sync(1);

        flags.set_update_dead_time_on_tez(1);
        flags.set_update_dead_time_on_tep(1);
        flags.set_update_dead_time_on_sync(1);

        OperatorConfig {
            flags,
            comparator_x: Default::default(), // SOC_MCPWM_COMPARATORS_PER_OPERATOR is 2 for ESP32 and ESP32-S3
            comparator_y: Default::default(),

            generator_a: NoGenCfg, // One generator per pin, with a maximum of two generators per Operator
            generator_b: NoGenCfg,
        }
    }
}

impl OperatorConfig<NoGenCfg, NoGenCfg> {
    fn cmp_x(self, config: ComparatorConfig) -> OperatorConfig<NoGenCfg, NoGenCfg> {
        OperatorConfig {
            flags: self.flags,
            comparator_x: config,
            comparator_y: self.comparator_y,

            generator_a: self.generator_a,
            generator_b: self.generator_b,
        }
    }

    fn cmp_y(self, config: ComparatorConfig) -> OperatorConfig<NoGenCfg, NoGenCfg> {
        OperatorConfig {
            flags: self.flags,
            comparator_x: self.comparator_x,
            comparator_y: config,

            generator_a: self.generator_a,
            generator_b: self.generator_b,
        }
    }
}

impl<GENB: OptionalGenCfg> OperatorConfig<NoGenCfg, GENB> {
    #[allow(clippy::type_complexity)]
    fn gen_a<P: OutputPin>(
        self,
        config: GeneratorConfig<GenA, P>,
    ) -> OperatorConfig<GeneratorConfig<GenA, P>, GENB> {
        OperatorConfig {
            flags: self.flags,
            comparator_x: self.comparator_x,
            comparator_y: self.comparator_y,

            generator_a: config,
            generator_b: self.generator_b,
        }
    }
}

impl<GENA: OptionalGenCfg> OperatorConfig<GENA, NoGenCfg> {
    #[allow(clippy::type_complexity)]
    fn gen_b<P: OutputPin>(
        self,
        config: GeneratorConfig<GenB, P>,
    ) -> OperatorConfig<GENA, GeneratorConfig<GenB, P>> {
        OperatorConfig {
            flags: self.flags,
            comparator_x: self.comparator_x,
            comparator_y: self.comparator_y,

            generator_a: self.generator_a,
            generator_b: config,
        }
    }
}
