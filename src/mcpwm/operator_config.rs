use esp_idf_sys::mcpwm_operator_config_t__bindgen_ty_1;

use crate::gpio::OutputPin;

use super::{
    comparator::{ComparatorConfig, NoCmpCfg, OptionalCmpCfg},
    generator::{CountingDirection, GenA, GenB, GeneratorConfig, NoGenCfg, OptionalGenCfg},
};

type DefaultGeneratorConfigA<PA> = GeneratorConfig<GenA, CountingDirection, CountingDirection, PA>;
type DefaultGeneratorConfigB<PB> = GeneratorConfig<GenB, CountingDirection, CountingDirection, PB>;

#[derive(Default)]
pub struct OperatorConfig<CMPX, CMPY, GENA, GENB> {
    // TODO: When, how and who should set the flags?
    // Should that be done automagically, manually or does some specific setting cover all cases?
    /// Flags for Operator
    pub(crate) flags: mcpwm_operator_config_t__bindgen_ty_1,

    /// Configuration for Comparator X
    pub(crate) comparator_x: CMPX,

    /// Configuration for Comparator Y
    pub(crate) comparator_y: CMPY,

    /// Configuration for Generator A
    pub(crate) generator_a: GENA,

    /// Configuration for Generator B
    pub(crate) generator_b: GENB,
}

impl OperatorConfig<NoCmpCfg, NoCmpCfg, NoGenCfg, NoGenCfg> {
    pub fn default<PA: OutputPin, PB: OutputPin>(
        pin_a: PA,
        pin_b: PB,
    ) -> OperatorConfig<
        ComparatorConfig,
        ComparatorConfig,
        DefaultGeneratorConfigA<PA>,
        DefaultGeneratorConfigB<PB>,
    > {
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
            comparator_x: NoCmpCfg, // SOC_MCPWM_COMPARATORS_PER_OPERATOR is 2 for ESP32 and ESP32-S3
            comparator_y: NoCmpCfg,

            generator_a: NoGenCfg, // One generator per pin, with a maximum of two generators per Operator
            generator_b: NoGenCfg,
        }
    }
}

impl<CMPY: OptionalCmpCfg> OperatorConfig<NoCmpCfg, CMPY, NoGenCfg, NoGenCfg> {
    fn cmp_x(
        self,
        config: ComparatorConfig,
    ) -> OperatorConfig<ComparatorConfig, CMPY, NoGenCfg, NoGenCfg> {
        OperatorConfig {
            flags: self.flags,
            comparator_x: config,
            comparator_y: self.comparator_y,

            generator_a: self.generator_a,
            generator_b: self.generator_b,
        }
    }
}

impl<CMPX: OptionalCmpCfg> OperatorConfig<CMPX, NoCmpCfg, NoGenCfg, NoGenCfg> {
    fn cmp_y(
        self,
        config: ComparatorConfig,
    ) -> OperatorConfig<CMPX, ComparatorConfig, NoGenCfg, NoGenCfg> {
        OperatorConfig {
            flags: self.flags,
            comparator_x: self.comparator_x,
            comparator_y: config,

            generator_a: self.generator_a,
            generator_b: self.generator_b,
        }
    }
}

impl<CMPX: OptionalCmpCfg, CMPY: OptionalCmpCfg, GENB: OptionalGenCfg>
    OperatorConfig<CMPX, CMPY, NoGenCfg, GENB>
{
    #[allow(clippy::type_complexity)]
    fn gen_a<P: OutputPin>(
        self,
        config: GeneratorConfig<GenA, CMPX::OnMatchCfg, CMPY::OnMatchCfg, P>,
    ) -> OperatorConfig<
        CMPX,
        CMPY,
        GeneratorConfig<GenA, CMPX::OnMatchCfg, CMPY::OnMatchCfg, P>,
        GENB,
    > {
        OperatorConfig {
            flags: self.flags,
            comparator_x: self.comparator_x,
            comparator_y: self.comparator_y,

            generator_a: config,
            generator_b: self.generator_b,
        }
    }
}

impl<CMPX: OptionalCmpCfg, CMPY: OptionalCmpCfg, GENA: OptionalGenCfg>
    OperatorConfig<CMPX, CMPY, GENA, NoGenCfg>
{
    #[allow(clippy::type_complexity)]
    fn gen_b<P: OutputPin>(
        self,
        config: GeneratorConfig<GenB, CMPX::OnMatchCfg, CMPY::OnMatchCfg, P>,
    ) -> OperatorConfig<
        CMPX,
        CMPY,
        GENA,
        GeneratorConfig<GenB, CMPX::OnMatchCfg, CMPY::OnMatchCfg, P>,
    > {
        OperatorConfig {
            flags: self.flags,
            comparator_x: self.comparator_x,
            comparator_y: self.comparator_y,

            generator_a: self.generator_a,
            generator_b: config,
        }
    }
}
