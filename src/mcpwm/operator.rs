use esp_idf_sys::{mcpwm_oper_handle_t, EspError};

use crate::{gpio::OutputPin, mcpwm::Group};

use super::{timer_connection::OptionalOutputPin, Duty};

use core::{ffi, marker::PhantomData};

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
    GEN_A: OptionalGenerator,
    GEN_B: OptionalGenerator,
> {
    _instance: OPERATOR<N, G>,
    handle: mcpwm_oper_handle_t,

    comparator_x: CMP_X, // SOC_MCPWM_COMPARATORS_PER_OPERATOR is 2 for ESP32 and ESP32-S3
    comparator_y: CMP_Y,

    generator_a: GEN_A, // One generator per pin, with a maximum of two generators per Operator
    generator_b: GEN_B,
    //deadtime: D
}

impl<const N: u8, G: Group> Operator<N, G, NoGen, NoGen, NoCmp, NoCmp> {
    pub fn default(instance: OPERATOR<N, G>) -> _ {
        Operator::custom(instance)
            .cmp_x(Default::default())
            .cmp_y(Default::default())
            .gen_a(Default::default())
            .gen_b(Default::default())
    }

    pub fn custom(instance: OPERATOR<N, G>) -> Self {
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

impl<
        const N: u8,
        G: Group,
    > Operator<N, G, NoGen, NoGen, NoCmp, CMP_Y>
{
    fn cmp_x(self, config: ComparatorConfig) -> Operator<N, G, NoGen, NoGen, Comparator, CMP_Y> {
        let cfg: mcpwm_comparator_config_t = todo!();
        let mut cmp = ptr::null_mut();
        esp!(mcpwm_new_comparator(self.handle, &cfg, &mut cmp)).unwrap();

        let comparator_x = Comparator(cmp);

        Operator {
            _instance: instance,
            handle,
            comparator_x,
            comparator_y: self.comparator_y,

            generator_a: self.generator_a,
            generator_b: self.generator_b,
        }
    }
}

impl<
        const N: u8,
        G: Group,
    > Operator<N, G, NoGen, NoGen, CMP_X, NoCmp>
{
    fn cmp_y(self, config: ComparatorConfig) -> Operator<N, G, NoGen, NoGen, CMP_X, Comparator> {
        let cfg: mcpwm_comparator_config_t = todo!();
        let mut cmp = ptr::null_mut();
        esp!(mcpwm_new_comparator(self.handle, &cfg, &mut cmp)).unwrap();

        let comparator_y = Comparator(cmp);

        Operator {
            _instance: instance,
            handle,
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


impl<
    const N: u8,
    G: Group,
    GEN_B: OptionalGen,
    CMP_X: OptionalCmp,
    CMP_Y: OptionalCmp,
> Operator<N, G, NoGen, GEN_B, NoCmp, NoCmp>
{
    fn gen_a(self, config: Into<GeneratorConf<GenA, CMP_X::OnMatchCfg, CMP_Y::OnMatchCfg>>) -> Operator<> {
        let cfg: mcpwm_generator_config_t = todo!();
        let mut gen = ptr::null_mut();
        esp!(mcpwm_new_generator(self.handle, &cfg, &gen)).unwrap();

        esp_err_t mcpwm_generator_set_actions_on_timer_event(mcpwm_gen_handle_t gen, mcpwm_gen_timer_event_action_t ev_act, ...);
        esp_err_t mcpwm_generator_set_actions_on_compare_event(mcpwm_gen_handle_t gen, mcpwm_gen_compare_event_action_t ev_act, ...)

        let generator_a = Generator(gen);

        Operator {
            _instance: instance,
            handle,
            comparator_x: self.comparator_x,
            comparator_y: self.comparator_y,

            generator_a,
            generator_b: self.generator_b,
        }
    }
}

impl<
    const N: u8,
    G: Group,
    GEN_A: OptionalGen,
    CMP_X: OptionalCmp,
    CMP_Y: OptionalCmp,
> Operator<N, G, GEN_A, NoGen>
{
    fn gen_b(self, config: Into<GeneratorConf<GenB, CMP_X::OnMatchCfg, CMP_Y::OnMatchCfg>>) -> Operator<> {
        let cfg: mcpwm_generator_config_t = todo!();
        let mut gen = ptr::null_mut();
        esp!(mcpwm_new_generator(self.handle, &cfg, &gen)).unwrap();

        esp_err_t mcpwm_generator_set_actions_on_timer_event(mcpwm_gen_handle_t gen, mcpwm_gen_timer_event_action_t ev_act, ...);
        esp_err_t mcpwm_generator_set_actions_on_compare_event(mcpwm_gen_handle_t gen, mcpwm_gen_compare_event_action_t ev_act, ...)

        let generator_b = Generator(gen);

        Operator {
            _instance: instance,
            handle,
            comparator_x: self.comparator_x,
            comparator_y: self.comparator_y,

            generator_a: self.generator_a,
            generator_b,
        }
    }
}

impl<const N: u8, G, GEN_A, GEN_B, CMP_Y> Operator<N, G, GEN_A, GEN_B, Comparator, CMP_Y>
where
    G: Group,
    GEN_A: OptionalGen,
    GEN_B: OptionalGen,
    CMP_Y: OptionalCmp,
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
        esp!(mcpwm_comparator_set_compare_value(&self.comparator_x.0, value.into()))
    }
}

impl<const N: u8, G, GEN_A, GEN_B, CMP_X> Operator<N, G, GEN_A, GEN_B, CMP_X, Comparator>
where
    G: Group,
    GEN_A: OptionalGen,
    GEN_B: OptionalGen,
    CMP_X: OptionalCmp,
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
        esp!(mcpwm_comparator_set_compare_value(&self.comparator_x.0, value.into()))
    }
}

pub trait OptionalOperator<const N: u8, G: Group> {}
impl<const N: u8, G: Group, PA: OptionalOutputPin, PB: OptionalOutputPin> OptionalOperator<N, G>
    for Operator<N, G, PA, PB>
{
}

pub struct NoOperator;
impl<const N: u8, G: Group> OptionalOperator<N, G> for NoOperator {}