
use esp_idf_sys::mcpwm_operator_config_t__bindgen_ty_1;

use crate::gpio::OutputPin;

use super::{
    comparator::{ComparatorConfig, NoCmpCfg, OptionalCmpCfg},
    generator::{GenA, GenB, GeneratorConfig, NoGenCfg, OptionalGenCfg, CountingDirection},
};

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
        GeneratorConfig<GenA, CountingDirection, CountingDirection, PA>,
        GeneratorConfig<GenB, CountingDirection, CountingDirection, PB>,
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

impl<CMPX, CMPY, GENA, GENB> OperatorConfig<CMPX, CMPY, GENA, GENB>
where
    CMPX: OptionalCmpCfg,
    CMPY: OptionalCmpCfg,
    GENA: OptionalGenCfg,
    GENB: OptionalGenCfg,
{
    /*fn set_update_dead_time_on_tez(mut self, update_dead_time_on_tez: bool) -> Self {
        self.flags
            .set_update_dead_time_on_tez(update_dead_time_on_tez.into());
        self
    }

    fn set_update_dead_time_on_tep(mut self, set_update_dead_time_on_tep: bool) -> Self {
        self.flags
            .set_update_dead_time_on_tep(set_update_dead_time_on_tep.into());
        self
    }

    fn set_update_dead_time_on_sync(mut self, update_dead_time_on_sync: bool) -> Self {
        self.flags
            .set_update_dead_time_on_sync(update_dead_time_on_sync.into());
        self
    }

    fn set_update_gen_action_on_tez(mut self, update_gen_action_on_tez: bool) -> Self {
        self.flags
            .set_update_gen_action_on_tez(update_gen_action_on_tez.into());
        self
    }

    fn set_update_gen_action_on_tep(mut self, set_update_gen_action_on_tep: bool) -> Self {
        self.flags
            .set_update_gen_action_on_tep(set_update_gen_action_on_tep.into());
        self
    }

    fn set_update_gen_action_on_sync(mut self, update_gen_action_on_sync: bool) -> Self {
        self.flags
            .set_update_gen_action_on_sync(update_gen_action_on_sync.into());
        self
    }*/
}

impl<CMPY: OptionalCmpCfg> OperatorConfig<NoCmpCfg, CMPY, NoGenCfg, NoGenCfg> {
    fn cmp_x(
        self,
        config: ComparatorConfig,
    ) -> OperatorConfig<ComparatorConfig, CMPY, NoGenCfg, NoGenCfg> {
        /*
        let mut flags: mcpwm_comparator_config_t__bindgen_ty_1 = Default::default();
        flags.set_update_cmp_on_tep(todo!());
        flags.set_update_cmp_on_tez(todo!());
        flags.set_update_cmp_on_sync(todo!());

        let cfg = mcpwm_comparator_config_t { flags };

        let mut cmp = ptr::null_mut();
        unsafe {
            esp!(mcpwm_new_comparator(self.handle, &cfg, &mut cmp)).unwrap();
        }
        let comparator_x = Comparator(cmp);*/

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
        /*let mut flags: mcpwm_comparator_config_t__bindgen_ty_1 = Default::default();
        flags.set_update_cmp_on_tep(todo!());
        flags.set_update_cmp_on_tez(todo!());
        flags.set_update_cmp_on_sync(todo!());

        let cfg = mcpwm_comparator_config_t { flags };
        let mut cmp = ptr::null_mut();
        unsafe {
            esp!(mcpwm_new_comparator(self.handle, &cfg, &mut cmp)).unwrap();
        }
        let comparator_y = Comparator(cmp);
        */

        OperatorConfig {
            flags: self.flags,
            comparator_x: self.comparator_x,
            comparator_y: config,

            generator_a: self.generator_a,
            generator_b: self.generator_b,
        }
    }
}

// TODO: Make sure that a generator config can only refer to comparators that are assigned to the operator
// TODO: Is there any point in letting the user provide the comparators or should two (the only two available
// for that operator in hardware) be automatically assigned in `Operator::new`?

impl<CMPX: OptionalCmpCfg, CMPY: OptionalCmpCfg, GENB: OptionalGenCfg>
    OperatorConfig<CMPX, CMPY, NoGenCfg, GENB>
{
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