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
    EspError,
};

use crate::{gpio::OutputPin, peripheral::Peripheral};

use super::{comparator::Comparator, Group};

pub enum ComparatorChannel {
    CmpX,
    CmpY,
}

pub struct Generator<'d, G> {
    pub(crate) _handle: mcpwm_gen_handle_t,
    _p: PhantomData<&'d mut ()>,
    _group: PhantomData<G>,
}

pub struct GeneratorConfig<'d> {
    pub(crate) flags: mcpwm_generator_config_t__bindgen_ty_1,
    pub(crate) on_matches_cmp_x: CountingDirection,
    pub(crate) on_matches_cmp_y: CountingDirection,
    pub(crate) on_is_empty: CountingDirection,
    pub(crate) on_is_full: CountingDirection,
    pub(crate) pin: i32,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> GeneratorConfig<'d> {
    pub(crate) unsafe fn init<G: Group>(
        self,
        operator_handle: mcpwm_oper_handle_t,
        cmp_x: &mut Comparator,
        cmp_y: &mut Comparator,
    ) -> Result<Generator<'d, G>, EspError> {
        let cfg = mcpwm_generator_config_t {
            gen_gpio_num: self.pin,
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
            esp!(mcpwm_new_generator(operator_handle, &cfg, &mut gen))?;

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
            ))?;

            cmp_x.configure(&mut *gen, self.on_matches_cmp_x)?;
            cmp_y.configure(&mut *gen, self.on_matches_cmp_y)?;
        }

        //
        Ok(Generator {
            _handle: gen,
            _p: PhantomData,
            _group: PhantomData,
        })
    }

    pub fn active_high(
        pin: impl Peripheral<P = impl OutputPin> + 'd,
        cmp_channel: ComparatorChannel,
    ) -> Self {
        let mut result: Self = GeneratorConfig::empty(pin);

        result.on_is_empty.counting_up = GeneratorAction::SetHigh;
        match cmp_channel {
            ComparatorChannel::CmpX => {
                result.on_matches_cmp_x.counting_up = GeneratorAction::SetLow
            }
            ComparatorChannel::CmpY => {
                result.on_matches_cmp_y.counting_up = GeneratorAction::SetLow
            }
        }
        result
    }

    pub fn active_low(
        pin: impl Peripheral<P = impl OutputPin> + 'd,
        cmp_channel: ComparatorChannel,
    ) -> Self {
        let mut result: Self = GeneratorConfig::empty(pin);

        result.on_is_empty.counting_up = GeneratorAction::SetLow;
        match cmp_channel {
            ComparatorChannel::CmpX => {
                result.on_matches_cmp_x.counting_up = GeneratorAction::SetHigh
            }
            ComparatorChannel::CmpY => {
                result.on_matches_cmp_y.counting_up = GeneratorAction::SetHigh
            }
        }
        result
    }
}

// TODO: Do we have any use for this?
impl<'d> GeneratorConfig<'d> {
    fn empty(pin: impl Peripheral<P = impl OutputPin> + 'd) -> Self {
        crate::into_ref!(pin);

        let mut flags: mcpwm_generator_config_t__bindgen_ty_1 = Default::default();
        flags.set_invert_pwm(0);
        flags.set_io_loop_back(0);

        GeneratorConfig {
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
            pin: pin.pin(),
            _p: PhantomData,
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
