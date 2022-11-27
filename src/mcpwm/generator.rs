use core::marker::PhantomData;

use esp_idf_sys::{
    mcpwm_gen_handle_t, mcpwm_generator_action_t, mcpwm_generator_action_t_MCPWM_GEN_ACTION_HIGH,
    mcpwm_generator_action_t_MCPWM_GEN_ACTION_KEEP, mcpwm_generator_action_t_MCPWM_GEN_ACTION_LOW,
    mcpwm_generator_action_t_MCPWM_GEN_ACTION_TOGGLE,
};

use crate::gpio::OutputPin;

pub struct NoGen;

impl OptionalGen for NoGen {}
pub trait OptionalGen {}

impl<P: OutputPin> OptionalGen for Generator<P> {}

pub trait GeneratorChannel {
    const IS_A: bool;
}

pub struct GenA;
impl GeneratorChannel for GenA {
    const IS_A: bool = true;
}

pub struct GenB;
impl GeneratorChannel for GenB {
    const IS_A: bool = false;
}

// TODO: Allow OptionalOutputPin?
pub struct Generator<P: OutputPin> {
    pub(crate) handle: mcpwm_gen_handle_t,
    pub(crate) pin: P,
}

#[derive(Clone)]
pub struct GeneratorConfig<G: GeneratorChannel, CMP_X, CMP_Y> {
    _channel: PhantomData<G>,
    pub(crate) invert: bool,
    pub(crate) on_matches_cmp_x: CMP_X,
    pub(crate) on_matches_cmp_y: CMP_Y,
    pub(crate) on_is_empty: CountingDirection,
    pub(crate) on_is_full: CountingDirection,
}

impl<G: GeneratorChannel> GeneratorConfig<G, CountingDirection, CountingDirection> {
    pub fn active_high() -> Self {
        let mut result: Self = GeneratorConfig::empty();

        result.on_is_empty.counting_up = GeneratorAction::SetHigh;
        if G::IS_A {
            result.on_matches_cmp_x.counting_up = GeneratorAction::SetLow;
        } else {
            result.on_matches_cmp_y.counting_up = GeneratorAction::SetLow;
        }
        result
    }

    pub fn active_low() -> Self {
        let mut result: Self = GeneratorConfig::empty();
        result.on_is_empty.counting_up = GeneratorAction::SetLow;
        if G::IS_A {
            result.on_matches_cmp_x.counting_up = GeneratorAction::SetHigh;
        } else {
            result.on_matches_cmp_y.counting_up = GeneratorAction::SetHigh;
        }
        result
    }
}

// TODO: Do we have any use for this?
impl<G, CMP_X, CMP_Y> GeneratorConfig<G, CMP_X, CMP_Y>
where
    G: GeneratorChannel,
    CMP_X: OnMatchCfg,
    CMP_Y: OnMatchCfg,
{
    fn empty() -> Self {
        GeneratorConfig {
            _channel: PhantomData,
            invert: false,
            on_matches_cmp_x: OnMatchCfg::empty(),
            on_matches_cmp_y: OnMatchCfg::empty(),
            on_is_empty: CountingDirection {
                counting_up: GeneratorAction::Nothing,
                counting_down: GeneratorAction::Nothing,
            },
            on_is_full: CountingDirection {
                counting_up: GeneratorAction::Nothing,
                counting_down: GeneratorAction::Nothing,
            },
        }
    }
}

pub struct NoCmpMatchConfig;

impl OnMatchCfg for NoCmpMatchConfig {
    fn empty() -> Self {
        NoCmpMatchConfig
    }
}

// TODO: Come up with better name
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CountingDirection {
    pub(crate) counting_up: GeneratorAction,
    pub(crate) counting_down: GeneratorAction,
}

impl CountingDirection {
    pub fn empty() -> Self {
        CountingDirection {
            counting_up: GeneratorAction::Nothing,
            counting_down: GeneratorAction::Nothing,
        }
    }
}

impl OnMatchCfg for CountingDirection {
    fn empty() -> Self {
        CountingDirection::empty()
    }
}

pub trait OnMatchCfg {
    fn empty() -> Self;
}

impl Into<mcpwm_generator_action_t> for GeneratorAction {
    fn into(self) -> mcpwm_generator_action_t {
        match self {
            GeneratorAction::Nothing => mcpwm_generator_action_t_MCPWM_GEN_ACTION_KEEP,
            GeneratorAction::SetLow => mcpwm_generator_action_t_MCPWM_GEN_ACTION_LOW,
            GeneratorAction::SetHigh => mcpwm_generator_action_t_MCPWM_GEN_ACTION_HIGH,
            GeneratorAction::Toggle => mcpwm_generator_action_t_MCPWM_GEN_ACTION_TOGGLE,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GeneratorAction {
    Nothing,
    SetLow,
    SetHigh,
    Toggle,
}
