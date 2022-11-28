use esp_idf_sys::{
    esp, mcpwm_comparator_set_compare_value, mcpwm_oper_handle_t, mcpwm_operator_config_t,
    mcpwm_operator_config_t__bindgen_ty_1, mcpwm_operator_connect_timer, mcpwm_timer_handle_t,
    EspError,
};

use crate::mcpwm::Group;

use super::{
    comparator::{Comparator, OptionalCmp, OptionalCmpCfg},
    generator::{OptionalGen, OptionalGenCfg},
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
    CMPX: OptionalCmp,
    CMPY: OptionalCmp,
    GENA: OptionalGen,
    GENB: OptionalGen,
> {
    _instance: OPERATOR<N, G>,
    _handle: mcpwm_oper_handle_t,

    comparator_x: CMPX, // SOC_MCPWM_COMPARATORS_PER_OPERATOR is 2 for ESP32 and ESP32-S3
    comparator_y: CMPY,

    _generator_a: GENA, // One generator per pin, with a maximum of two generators per Operator
    _generator_b: GENB,
    //deadtime: D
}

pub unsafe fn new<const N: u8, G, CMPX, CMPY, GENA, GENB>(
    instance: OPERATOR<N, G>,
    timer_handle: mcpwm_timer_handle_t,
    cfg: OperatorConfig<CMPX, CMPY, GENA, GENB>,
) -> Operator<N, G, CMPX::Cmp, CMPY::Cmp, GENA::Gen, GENB::Gen>
where
    G: Group,
    CMPX: OptionalCmpCfg,
    CMPY: OptionalCmpCfg,

    GENA: OptionalGenCfg,
    GENB: OptionalGenCfg,
{
    let mut handle = ptr::null_mut();
    let mut flags: mcpwm_operator_config_t__bindgen_ty_1 = Default::default();

    // TODO: What should these be set to?
    flags.set_update_gen_action_on_tez(1);
    flags.set_update_gen_action_on_tep(1);
    flags.set_update_gen_action_on_sync(1);

    flags.set_update_dead_time_on_tez(1);
    flags.set_update_dead_time_on_tep(1);
    flags.set_update_dead_time_on_sync(1);

    let config = mcpwm_operator_config_t {
        group_id: G::ID,
        flags,
    };

    unsafe {
        esp!(esp_idf_sys::mcpwm_new_operator(&config, &mut handle,)).unwrap();
    }

    let mut comparator_x = unsafe { cfg.comparator_x.init(handle) };
    let mut comparator_y = unsafe { cfg.comparator_y.init(handle) };

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

impl<const N: u8, G, GENA, GENB, CMPY> Operator<N, G, Comparator, CMPY, GENA, GENB>
where
    G: Group,
    CMPY: OptionalCmp,
    GENA: OptionalGen,
    GENB: OptionalGen,
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

impl<const N: u8, G, GENA, GENB, CMPX> Operator<N, G, CMPX, Comparator, GENA, GENB>
where
    G: Group,
    CMPX: OptionalCmp,
    GENA: OptionalGen,
    GENB: OptionalGen,
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
impl<const N: u8, G, CMPX, CMPY, GENA, GENB> OptionalOperator<N, G>
    for Operator<N, G, CMPX, CMPY, GENA, GENB>
where
    G: Group,
    CMPX: OptionalCmp,
    CMPY: OptionalCmp,
    GENA: OptionalGen,
    GENB: OptionalGen,
{
}

pub struct NoOperator;
impl<const N: u8, G: Group> OptionalOperator<N, G> for NoOperator {}
