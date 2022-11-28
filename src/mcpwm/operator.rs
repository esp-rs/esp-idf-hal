use esp_idf_sys::{
    esp, mcpwm_comparator_config_t, mcpwm_comparator_config_t__bindgen_ty_1,
    mcpwm_comparator_set_compare_value, mcpwm_gen_timer_event_action_t, mcpwm_generator_config_t,
    mcpwm_generator_config_t__bindgen_ty_1, mcpwm_generator_set_actions_on_compare_event,
    mcpwm_generator_set_actions_on_timer_event, mcpwm_new_comparator, mcpwm_new_generator,
    mcpwm_oper_handle_t, mcpwm_operator_config_t, mcpwm_operator_config_t__bindgen_ty_1,
    mcpwm_operator_connect_timer, mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
    mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP, mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
    mcpwm_timer_event_t_MCPWM_TIMER_EVENT_FULL, mcpwm_timer_handle_t, EspError,
};

use crate::mcpwm::Group;

use super::{
    comparator::{Comparator, ComparatorConfig, NoCmp, OptionalCmp, OptionalCmpCfg},
    generator::{GenA, GenB, GenInit, NoGen, OptionalGen, OptionalGenCfg},
    OperatorConfig,
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
    _handle: mcpwm_oper_handle_t,

    comparator_x: CMP_X, // SOC_MCPWM_COMPARATORS_PER_OPERATOR is 2 for ESP32 and ESP32-S3
    comparator_y: CMP_Y,

    _generator_a: GEN_A, // One generator per pin, with a maximum of two generators per Operator
    _generator_b: GEN_B,
    //deadtime: D
}

pub unsafe fn new<const N: u8, G, CMP_X, CMP_Y, GEN_A, GEN_B>(
    instance: OPERATOR<N, G>,
    timer_handle: mcpwm_timer_handle_t,
    cfg: OperatorConfig<CMP_X, CMP_Y, GEN_A, GEN_B>,
) -> Operator<N, G, CMP_X::Cmp, CMP_Y::Cmp, GEN_A::Gen, GEN_B::Gen>
where
    G: Group,
    CMP_X: OptionalCmpCfg,
    CMP_Y: OptionalCmpCfg,

    GEN_A: OptionalGenCfg,
    GEN_B: OptionalGenCfg,
{
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
        esp!(esp_idf_sys::mcpwm_new_operator(&config, &mut handle,)).unwrap();
    }

    let comparator_x = unsafe { cfg.comparator_x.init(handle) };
    let comparator_y = unsafe { cfg.comparator_y.init(handle) };

    let generator_a = unsafe {
        cfg.generator_a.init(
            handle,
            comparator_x.get_comparator_mut(),
            comparator_y.get_comparator_mut(),
        )
    };
    let generator_b = unsafe {
        cfg.generator_b.init(
            handle,
            comparator_x.get_comparator_mut(),
            comparator_y.get_comparator_mut(),
        )
    };

    // Connect operator to timer
    unsafe {
        esp!(mcpwm_operator_connect_timer(handle, timer_handle)).unwrap();
    }

    Operator {
        _instance: instance,
        _handle: handle,
        comparator_x,
        comparator_y,

        _generator_a: generator_a,
        _generator_b: generator_b,
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
            esp!(mcpwm_new_comparator(self._handle, &cfg, &mut cmp)).unwrap();
        }
        let comparator_x = Comparator(cmp);

        Operator {
            _instance: self._instance,
            _handle: self._handle,
            comparator_x,
            comparator_y: self.comparator_y,

            _generator_a: self._generator_a,
            _generator_b: self._generator_b,
        }
    }
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
}

pub struct NoOperator;
impl<const N: u8, G: Group> OptionalOperator<N, G> for NoOperator {}
