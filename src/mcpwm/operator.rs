use esp_idf_sys::{
    esp, mcpwm_comparator_config_t, mcpwm_comparator_config_t__bindgen_ty_1,
    mcpwm_comparator_set_compare_value, mcpwm_gen_timer_event_action_t, mcpwm_generator_config_t,
    mcpwm_generator_config_t__bindgen_ty_1, mcpwm_generator_set_actions_on_compare_event,
    mcpwm_generator_set_actions_on_timer_event, mcpwm_new_comparator, mcpwm_new_generator,
    mcpwm_oper_handle_t, mcpwm_operator_config_t, mcpwm_operator_config_t__bindgen_ty_1,
    mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
    mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP, mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
    mcpwm_timer_event_t_MCPWM_TIMER_EVENT_FULL, EspError,
};

use crate::{gpio::OutputPin, mcpwm::Group};

use super::{
    comparator::{Comparator, ComparatorConfig, NoCmp, OptionalCmp},
    generator::{GenA, GenB, Generator, GeneratorConfig, NoGen, OptionalGen, GeneratorChannel},
    timer_connection::OptionalOutputPin,
};

use core::{marker::PhantomData, ptr};

pub struct OPERATOR<const N: u8, G: Group> {
    _ptr: PhantomData<*const ()>,
    _group: PhantomData<G>,
}

impl<const N: u8, G: Group> OPERATOR<N, G> {
    /// # Safety
    ///
    /// Care should be taken not to instnatiate this peripheralinstance, if it is already instantiated and used elsewhere
    #[inline(always)]
    pub unsafe fn new() -> Self {
        Self {
            _ptr: PhantomData,
            _group: PhantomData,
        }
    }
}

unsafe impl<const N: u8, G: Group> Send for OPERATOR<N, G> {}

impl<const N: u8, G: Group> crate::peripheral::sealed::Sealed for OPERATOR<N, G> {}

impl<const N: u8, G: Group> crate::peripheral::Peripheral for OPERATOR<N, G> {
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self { ..*self }
    }
}

// TODO: How do we want syncing to fit in to this?
// TODO: How do we want carrier to fit into this?
// TODO: How do we want capture to fit into this?

/// Motor Control operator abstraction
///
/// Every Motor Control module has three operators. Every operator can generate two output signals called A and B.
/// A and B share the same timer and thus frequency and phase but can have induvidual duty set.
pub struct Operator<
    const N: u8,
    G: Group,
    CMP_X: OptionalCmp,
    CMP_Y: OptionalCmp,
    GEN_A: OptionalGen,
    GEN_B: OptionalGen,
> {
    _instance: OPERATOR<N, G>,
    handle: mcpwm_oper_handle_t,

    comparator_x: CMP_X, // SOC_MCPWM_COMPARATORS_PER_OPERATOR is 2 for ESP32 and ESP32-S3
    comparator_y: CMP_Y,

    generator_a: GEN_A, // One generator per pin, with a maximum of two generators per Operator
    generator_b: GEN_B,
    //deadtime: D
}

impl<const N: u8, G: Group> Operator<N, G, NoCmp, NoCmp, NoGen, NoGen> {
    pub fn default<PA: OutputPin, PB: OutputPin>(instance: OPERATOR<N, G>, pin_a: PA, pin_b: PB, cfg: OperatorConfig) -> Operator<N, G, Comparator, Comparator, Generator<PA>, Generator<PB>> {
        Operator::custom(instance, cfg)
            .cmp_x(Default::default())
            .cmp_y(Default::default())
            .gen_a(pin_a, GeneratorConfig::active_high())
            .gen_b(pin_b, GeneratorConfig::active_high())
    }

    pub fn custom(instance: OPERATOR<N, G>, cfg: OperatorConfig) -> Self {
        let mut handle = ptr::null_mut();
        let mut flags: mcpwm_operator_config_t__bindgen_ty_1 = Default::default();

        flags.set_update_gen_action_on_tez(todo!());
        flags.set_update_gen_action_on_tep(todo!());
        flags.set_update_gen_action_on_sync(todo!());

        flags.set_update_dead_time_on_tez(todo!());
        flags.set_update_dead_time_on_tep(todo!());
        flags.set_update_dead_time_on_sync(todo!());
        
        let config = mcpwm_operator_config_t {
            group_id: G::ID,
            flags,
        };

        unsafe {
            esp!(esp_idf_sys::mcpwm_new_operator(
                &config,
                &mut handle,
            ))
            .unwrap();
        }

        Operator {
            _instance: instance,
            handle,
            comparator_x: NoCmp, // SOC_MCPWM_COMPARATORS_PER_OPERATOR is 2 for ESP32 and ESP32-S3
            comparator_y: NoCmp,

            generator_a: NoGen, // One generator per pin, with a maximum of two generators per Operator
            generator_b: NoGen,
        }
    }
}

impl<const N: u8, CMP_Y: OptionalCmp, G: Group> Operator<N, G, NoCmp, CMP_Y, NoGen, NoGen> {
    fn cmp_x(self, config: ComparatorConfig) -> Operator<N, G, Comparator, CMP_Y, NoGen, NoGen> {
        let mut flags: mcpwm_comparator_config_t__bindgen_ty_1 = Default::default();
        flags.set_update_cmp_on_tep(todo!());
        flags.set_update_cmp_on_tez(todo!());
        flags.set_update_cmp_on_sync(todo!());

        let cfg = mcpwm_comparator_config_t { flags };
        let mut cmp = ptr::null_mut();
        unsafe {
            esp!(mcpwm_new_comparator(self.handle, &cfg, &mut cmp)).unwrap();
        }
        let comparator_x = Comparator(cmp);

        Operator {
            _instance: self._instance,
            handle: self.handle,
            comparator_x,
            comparator_y: self.comparator_y,

            generator_a: self.generator_a,
            generator_b: self.generator_b,
        }
    }
}

impl<const N: u8, CMP_X: OptionalCmp, G: Group> Operator<N, G, CMP_X, NoCmp, NoGen, NoGen> {
    fn cmp_y(self, config: ComparatorConfig) -> Operator<N, G, CMP_X, Comparator, NoGen, NoGen> {
        let mut flags: mcpwm_comparator_config_t__bindgen_ty_1 = Default::default();
        flags.set_update_cmp_on_tep(todo!());
        flags.set_update_cmp_on_tez(todo!());
        flags.set_update_cmp_on_sync(todo!());

        let cfg = mcpwm_comparator_config_t { flags };
        let mut cmp = ptr::null_mut();
        unsafe {
            esp!(mcpwm_new_comparator(self.handle, &cfg, &mut cmp)).unwrap();
        }

        let comparator_y = Comparator(cmp);

        Operator {
            _instance: self._instance,
            handle: self.handle,
            comparator_x: self.comparator_x,
            comparator_y,

            generator_a: self.generator_a,
            generator_b: self.generator_b,
        }
    }
}

// TODO: Make sure that a generator config can only refer to comparators that are assigned to the operator
// TODO: Is there any point in letting the user provide the comparators or should two (the only two available
// for that operator in hardware) be automatically assigned in `Operator::new`?

impl<const N: u8, G: Group, CMP_X: OptionalCmp, CMP_Y: OptionalCmp, GEN_B: OptionalGen,>
    Operator<N, G, CMP_X, CMP_Y, NoGen, GEN_B>
{
    fn gen_a<P: OutputPin>(
        mut self,
        pin: P,
        config: GeneratorConfig<GenA, CMP_X::OnMatchCfg, CMP_Y::OnMatchCfg>,
    ) -> Operator<N, G, CMP_X, CMP_Y, Generator<P>, GEN_B> {
        let generator_a = setup_generator(
            &mut self.handle,
            &mut self.comparator_x,
            &mut self.comparator_y,
            pin,
            config,
        );

        Operator {
            _instance: self._instance,
            handle: self.handle,
            comparator_x: self.comparator_x,
            comparator_y: self.comparator_y,

            generator_a,
            generator_b: self.generator_b,
        }
    }
}

impl<const N: u8, G: Group, CMP_X: OptionalCmp, CMP_Y: OptionalCmp, GEN_A: OptionalGen>
    Operator<N, G, CMP_X, CMP_Y, GEN_A, NoGen>
{
    fn gen_b<P: OutputPin>(
        mut self,
        pin: P,
        config: GeneratorConfig<GenB, CMP_X::OnMatchCfg, CMP_Y::OnMatchCfg>,
    ) -> Operator<N, G, CMP_X, CMP_Y, GEN_A, Generator<P>> {
        let generator_b = setup_generator(
            &mut self.handle,
            &mut self.comparator_x,
            &mut self.comparator_y,
            pin,
            config,
        );

        Operator {
            _instance: self._instance,
            handle: self.handle,
            comparator_x: self.comparator_x,
            comparator_y: self.comparator_y,

            generator_a: self.generator_a,
            generator_b,
        }
    }
}

fn setup_generator<G: GeneratorChannel, CMP_X: OptionalCmp, CMP_Y: OptionalCmp, P: OutputPin>(
    operator: &mut mcpwm_oper_handle_t,
    comparator_x: &mut CMP_X,
    comparator_y: &mut CMP_Y,
    pin: P,
    config: GeneratorConfig<G, CMP_X::OnMatchCfg, CMP_Y::OnMatchCfg>,
) -> Generator<P> {
    let mut flags: mcpwm_generator_config_t__bindgen_ty_1 = Default::default();
    flags.set_invert_pwm(config.invert.into());
    flags.set_io_loop_back(0);
    let cfg = mcpwm_generator_config_t {
        gen_gpio_num: pin.pin(),
        flags,
    };
    let mut gen = ptr::null_mut();
    unsafe {
        esp!(mcpwm_new_generator(*operator, &cfg, &mut gen)).unwrap();

        // TODO: "must be terminated by MCPWM_GEN_TIMER_EVENT_ACTION_END()"
        esp!(mcpwm_generator_set_actions_on_timer_event(
            gen,
            mcpwm_gen_timer_event_action_t {
                direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
                event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
                action: config.on_is_empty.counting_up.into(),
            }
        )).unwrap();
        esp!(mcpwm_generator_set_actions_on_timer_event(
            gen,
            mcpwm_gen_timer_event_action_t {
                direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
                event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
                action: config.on_is_empty.counting_down.into(),
            }
        )).unwrap();
        esp!(mcpwm_generator_set_actions_on_timer_event(
            gen,
            mcpwm_gen_timer_event_action_t {
                direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
                event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_FULL,
                action: config.on_is_full.counting_up.into(),
            }
        )).unwrap();
        esp!(mcpwm_generator_set_actions_on_timer_event(
            gen,
            mcpwm_gen_timer_event_action_t {
                direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
                event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_FULL,
                action: config.on_is_full.counting_down.into(),
            }
        )).unwrap();

        comparator_x._configure(&mut *gen, config.on_matches_cmp_x);
        comparator_y._configure(&mut *gen, config.on_matches_cmp_y);
    }

    Generator { handle: gen, pin }
}

impl<const N: u8, G, GEN_A, GEN_B, CMP_Y> Operator<N, G, Comparator, CMP_Y, GEN_A, GEN_B>
where
    G: Group,
    CMP_Y: OptionalCmp,
    GEN_A: OptionalGen,
    GEN_B: OptionalGen,
{
    // TODO: Note that this is the comparator we are affecting, not the generator. Generator A may not necessarily have
    // anything to do with comparator A. How do we best convay that? Should we call them Generator X/Y and Comparator A/B?
    //
    // Again should we always provide two comparators? That would make both set/get_duty_a/b always available... Should we
    // instead let the user only provide only as few or many (max 2 as of ESP32/ESP32-S3) as they want. And depending on
    // expose only the corresponding set/get_duty?
    //
    // Once again to clarify, set_duty affects the comparator. The generators(booth or none) may then choose to use that
    // event, as well as timer events, to change the level of the pin.
    //
    /// Get compare value, often times same as the duty for output A.
    ///
    /// See `Self::set_compare_value_x` for more info
    pub fn get_compare_value_x(&self) -> u16 {
        todo!()
    }

    /// Set compare value, often times same as the duty for output A.
    ///
    /// Depending on how the generators are configured this is, using the most common configuration, the duty of output A.
    /// `value` is from the range 0 to timers peak value. However do note that if using a custom configuration this might
    /// control something else like for example the phase. Look into Generator::TODO for more details
    ///
    /// TODO: what about CountMode::UpDown?
    ///
    /// NOTE: The compare value shouldn’t exceed timer’s count peak, otherwise, the compare event will never got triggered.
    pub fn set_compare_value_x(&mut self, value: u16) -> Result<(), EspError> {
        unsafe {
            esp!(mcpwm_comparator_set_compare_value(
                self.comparator_x.0,
                value.into()
            ))
        }
    }
}

impl<const N: u8, G, GEN_A, GEN_B, CMP_X> Operator<N, G, CMP_X, Comparator, GEN_A, GEN_B>
where
    G: Group,
    CMP_X: OptionalCmp,
    GEN_A: OptionalGen,
    GEN_B: OptionalGen,
{
    /// Get compare value, often times same as the duty for output B.
    ///
    /// See `Self::set_compare_value_x` for more info
    pub fn get_compare_value_y(&self) -> u16 {
        todo!()
    }

    /// Set compare value, often times same as the duty for output A.
    ///
    /// Depending on how the generators are configured this is, using the most common configuration, the duty of output A.
    /// `value` is from the range 0 to timers peak value. However do note that if using a custom configuration this might
    /// control something else like for example the phase. Look into Generator::TODO for more details
    ///
    /// TODO: what about CountMode::UpDown?
    ///
    /// NOTE: The compare value shouldn’t exceed timer’s count peak, otherwise, the compare event will never got triggered.
    pub fn set_compare_value_y(&mut self, value: u16) -> Result<(), EspError> {
        unsafe {
            esp!(mcpwm_comparator_set_compare_value(
                self.comparator_y.0,
                value.into()
            ))
        }
    }
}

pub trait OptionalOperator<const N: u8, G: Group> {}
impl<const N: u8, G, CMP_X, CMP_Y, GEN_A, GEN_B> OptionalOperator<N, G>
    for Operator<N, G, CMP_X, CMP_Y, GEN_A, GEN_B>
where
    G: Group,
    CMP_X: OptionalCmp,
    CMP_Y: OptionalCmp,
    GEN_A: OptionalGen,
    GEN_B: OptionalGen,
{
    // TODO:
}

pub struct NoOperator;
impl<const N: u8, G: Group> OptionalOperator<N, G> for NoOperator {}

#[derive(Default)]
pub struct OperatorConfig {
    _todo: (),
}
