use core::{marker::PhantomData, ptr};

use esp_idf_sys::{
    esp, mcpwm_gen_handle_t, mcpwm_gen_timer_event_action_t, mcpwm_generator_action_t,
    mcpwm_generator_action_t_MCPWM_GEN_ACTION_HIGH, mcpwm_generator_action_t_MCPWM_GEN_ACTION_KEEP,
    mcpwm_generator_action_t_MCPWM_GEN_ACTION_LOW,
    mcpwm_generator_action_t_MCPWM_GEN_ACTION_TOGGLE, mcpwm_generator_config_t,
    mcpwm_generator_config_t__bindgen_ty_1, mcpwm_generator_set_actions_on_timer_event,
    mcpwm_new_generator, mcpwm_oper_handle_t, mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
    mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP, mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
    mcpwm_timer_event_t_MCPWM_TIMER_EVENT_FULL,
};

use crate::gpio::OutputPin;

use super::{
    comparator::OptionalCmp,
    timer_connection::{NoPin, OptionalOutputPin},
};

pub struct NoGen;

impl OptionalGen for NoGen {}
pub trait OptionalGen {}

impl<P: OutputPin> OptionalGen for Generator<P> {}

pub trait ToGenCfg {
    type Cfg: OptionalGenCfg;
}

impl<G: GeneratorChannel, CMP_X: OnMatchCfg, CMP_Y: OnMatchCfg, P: OutputPin> ToGenCfg
    for (CMP_X, CMP_Y, G, Generator<P>)
{
    type Cfg = GeneratorConfig<G, CMP_X, CMP_Y, P>;
}

impl<G: GeneratorChannel, CMP_X: OnMatchCfg, CMP_Y: OnMatchCfg> ToGenCfg for (CMP_X, CMP_Y, G, NoGenCfg) {
    type Cfg = NoGenCfg;
}

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

pub struct GeneratorConfig<G: GeneratorChannel, CMP_X, CMP_Y, P> {
    _channel: PhantomData<G>,
    pub(crate) invert: bool,
    pub(crate) on_matches_cmp_x: CMP_X,
    pub(crate) on_matches_cmp_y: CMP_Y,
    pub(crate) on_is_empty: CountingDirection,
    pub(crate) on_is_full: CountingDirection,
    pub(crate) pin: P,
}

pub struct NoGenCfg;

pub trait OptionalGenCfg {}

impl OptionalGenCfg for NoGenCfg {}

impl<G: GeneratorChannel, CMP_X: OnMatchCfg, CMP_Y: OnMatchCfg, P> OptionalGenCfg
    for GeneratorConfig<G, CMP_X, CMP_Y, P>
{
}

pub trait GenInit {
    type Gen: OptionalGen;

    /// This is only to be used internally by esp-idf-hal
    unsafe fn init(self, operator_handle: mcpwm_oper_handle_t) -> Self::Gen;
}

impl<CMP_X, CMP_Y> GenInit
    for (
        &mut CMP_X,
        &mut CMP_Y,
        NoGenCfg,
    )
where
    CMP_X: OptionalCmp,
    CMP_Y: OptionalCmp,
{
    type Gen = NoGen;

    unsafe fn init(self, operator_handle: mcpwm_oper_handle_t) -> Self::Gen {
        NoGen
    }
}

impl<G: GeneratorChannel, CMP_X, CMP_Y, P: OutputPin> GenInit
    for (
        &mut CMP_X,
        &mut CMP_Y,
        GeneratorConfig<G, CMP_X::OnMatchCfg, CMP_Y::OnMatchCfg, P>,
    )
where
    CMP_X: OptionalCmp,
    CMP_Y: OptionalCmp,
{
    type Gen = Generator<P>;

    unsafe fn init(mut self, operator_handle: mcpwm_oper_handle_t) -> Generator<P> {
        let (cmp_x, cmp_y, generator_config) = self;

        let cfg = mcpwm_generator_config_t {
            gen_gpio_num: generator_config.pin.pin(),
            flags: todo!(),//generator_config.flags,
        };
        let mut gen = ptr::null_mut();
        unsafe {
            esp!(mcpwm_new_generator(operator_handle, &cfg, &mut gen)).unwrap();

            // TODO: "must be terminated by MCPWM_GEN_TIMER_EVENT_ACTION_END()"
            esp!(mcpwm_generator_set_actions_on_timer_event(
                gen,
                mcpwm_gen_timer_event_action_t {
                    direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
                    event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
                    action: generator_config.on_is_empty.counting_up.into(),
                }
            ))
            .unwrap();
            esp!(mcpwm_generator_set_actions_on_timer_event(
                gen,
                mcpwm_gen_timer_event_action_t {
                    direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
                    event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
                    action: generator_config.on_is_empty.counting_down.into(),
                }
            ))
            .unwrap();
            esp!(mcpwm_generator_set_actions_on_timer_event(
                gen,
                mcpwm_gen_timer_event_action_t {
                    direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
                    event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_FULL,
                    action: generator_config.on_is_full.counting_up.into(),
                }
            ))
            .unwrap();
            esp!(mcpwm_generator_set_actions_on_timer_event(
                gen,
                mcpwm_gen_timer_event_action_t {
                    direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
                    event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_FULL,
                    action: generator_config.on_is_full.counting_down.into(),
                }
            ))
            .unwrap();

            cmp_x._configure(&mut *gen, generator_config.on_matches_cmp_x);
            cmp_y._configure(&mut *gen, generator_config.on_matches_cmp_y);
        }

        Generator {
            handle: gen,
            pin: generator_config.pin,
        }
    }
}

impl<G: GeneratorChannel, P> GeneratorConfig<G, CountingDirection, CountingDirection, P> {
    pub fn active_high(pin: P) -> Self {
        let mut result: Self = GeneratorConfig::empty(pin);

        result.on_is_empty.counting_up = GeneratorAction::SetHigh;
        if G::IS_A {
            result.on_matches_cmp_x.counting_up = GeneratorAction::SetLow;
        } else {
            result.on_matches_cmp_y.counting_up = GeneratorAction::SetLow;
        }
        result
    }

    pub fn active_low(pin: P) -> Self {
        let mut result: Self = GeneratorConfig::empty(pin);
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
impl<G, CMP_X, CMP_Y, P> GeneratorConfig<G, CMP_X, CMP_Y, P>
where
    G: GeneratorChannel,
    CMP_X: OnMatchCfg,
    CMP_Y: OnMatchCfg,
{
    fn empty(pin: P) -> Self {
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
            pin,
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
