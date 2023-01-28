use core::{marker::PhantomData, ptr};

use esp_idf_sys::{
    esp, mcpwm_gen_handle_t, mcpwm_gen_timer_event_action_t, mcpwm_generator_action_t,
    mcpwm_generator_action_t_MCPWM_GEN_ACTION_HIGH, mcpwm_generator_action_t_MCPWM_GEN_ACTION_KEEP,
    mcpwm_generator_action_t_MCPWM_GEN_ACTION_LOW,
    mcpwm_generator_action_t_MCPWM_GEN_ACTION_TOGGLE, mcpwm_generator_config_t,
    mcpwm_generator_config_t__bindgen_ty_1, mcpwm_new_generator, mcpwm_oper_handle_t,
    mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
    mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP, mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
    mcpwm_timer_event_t_MCPWM_TIMER_EVENT_FULL, mcpwm_timer_event_t_MCPWM_TIMER_EVENT_INVALID,
};

use crate::gpio::OutputPin;

use super::comparator::Comparator;

pub struct NoGen;

impl OptionalGen for NoGen {}
pub trait OptionalGen {}

impl<G, P> OptionalGen for Generator<G, P>
where
    G: GeneratorChannel,
    P: OutputPin,
{
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
pub struct Generator<G, P: OutputPin> {
    channel: PhantomData<G>,
    pub(crate) _handle: mcpwm_gen_handle_t,
    pub(crate) _pin: P,
}

pub struct GeneratorConfig<G: GeneratorChannel, P> {
    _channel: PhantomData<G>,
    pub(crate) flags: mcpwm_generator_config_t__bindgen_ty_1,
    pub(crate) on_matches_cmp_x: CountingDirection,
    pub(crate) on_matches_cmp_y: CountingDirection,
    pub(crate) on_is_empty: CountingDirection,
    pub(crate) on_is_full: CountingDirection,
    pub(crate) pin: P,
}

pub struct NoGenCfg;

pub trait OptionalGenCfg {
    type Gen: OptionalGen;

    /// This is only to be used internally by esp-idf-hal
    unsafe fn init(
        self,
        operator_handle: mcpwm_oper_handle_t,
        cmp_x: &mut Comparator,
        cmp_y: &mut Comparator,
    ) -> Self::Gen;
}

impl OptionalGenCfg for NoGenCfg {
    type Gen = NoGen;

    unsafe fn init(
        self,
        _operator_handle: mcpwm_oper_handle_t,
        _cmp_x: &mut Comparator,
        _cmp_y: &mut Comparator,
    ) -> NoGen {
        NoGen
    }
}

impl<G: GeneratorChannel, P: OutputPin> OptionalGenCfg for GeneratorConfig<G, P> {
    type Gen = Generator<G, P>;

    unsafe fn init(
        self,
        operator_handle: mcpwm_oper_handle_t,
        cmp_x: &mut Comparator,
        cmp_y: &mut Comparator,
    ) -> Self::Gen {
        let cfg = mcpwm_generator_config_t {
            gen_gpio_num: self.pin.pin(),
            flags: self.flags,
        };
        let mut gen = ptr::null_mut();

        extern "C" {
            fn mcpwm_generator_set_actions_on_timer_event(
                gen: mcpwm_gen_handle_t,
                ev_act0: mcpwm_gen_timer_event_action_t,
                ev_act1: mcpwm_gen_timer_event_action_t,
                ev_act2: mcpwm_gen_timer_event_action_t,
                ev_act3: mcpwm_gen_timer_event_action_t,

                ev_act_end: mcpwm_gen_timer_event_action_t,
            ) -> esp_idf_sys::esp_err_t;
        }

        unsafe {
            esp!(mcpwm_new_generator(operator_handle, &cfg, &mut gen)).unwrap();

            esp!(mcpwm_generator_set_actions_on_timer_event(
                gen,
                mcpwm_gen_timer_event_action_t {
                    direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
                    event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
                    action: self.on_is_empty.counting_up.into(),
                },
                mcpwm_gen_timer_event_action_t {
                    direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
                    event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
                    action: self.on_is_empty.counting_down.into(),
                },
                mcpwm_gen_timer_event_action_t {
                    direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
                    event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_FULL,
                    action: self.on_is_full.counting_up.into(),
                },
                mcpwm_gen_timer_event_action_t {
                    direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
                    event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_FULL,
                    action: self.on_is_full.counting_down.into(),
                },
                mcpwm_gen_timer_event_action_t {
                    // <-- This marks the last argument in the variadic list
                    event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_INVALID,
                    ..Default::default()
                }
            ))
            .unwrap();

            cmp_x.configure(&mut *gen, self.on_matches_cmp_x);
            cmp_y.configure(&mut *gen, self.on_matches_cmp_y);
        }

        Generator {
            channel: PhantomData,
            _handle: gen,
            _pin: self.pin,
        }
    }
}

impl<G: GeneratorChannel, P> GeneratorConfig<G, P> {
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
impl<G, P> GeneratorConfig<G, P>
where
    G: GeneratorChannel,
{
    fn empty(pin: P) -> Self {
        let mut flags: mcpwm_generator_config_t__bindgen_ty_1 = Default::default();
        flags.set_invert_pwm(0);
        flags.set_io_loop_back(0);

        GeneratorConfig {
            _channel: PhantomData,
            flags,
            on_matches_cmp_x: CountingDirection::empty(),
            on_matches_cmp_y: CountingDirection::empty(),
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

// TODO: Come up with better name
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

impl From<GeneratorAction> for mcpwm_generator_action_t {
    fn from(val: GeneratorAction) -> Self {
        match val {
            GeneratorAction::Nothing => mcpwm_generator_action_t_MCPWM_GEN_ACTION_KEEP,
            GeneratorAction::SetLow => mcpwm_generator_action_t_MCPWM_GEN_ACTION_LOW,
            GeneratorAction::SetHigh => mcpwm_generator_action_t_MCPWM_GEN_ACTION_HIGH,
            GeneratorAction::Toggle => mcpwm_generator_action_t_MCPWM_GEN_ACTION_TOGGLE,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeneratorAction {
    Nothing,
    SetLow,
    SetHigh,
    Toggle,
}
