use core::ptr;

use esp_idf_sys::{
    esp, mcpwm_cmpr_handle_t, mcpwm_comparator_config_t, mcpwm_comparator_config_t__bindgen_ty_1,
    mcpwm_gen_t, mcpwm_generator_set_actions_on_compare_event, mcpwm_new_comparator,
    mcpwm_oper_handle_t, mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
    mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
};

use super::generator::{CountingDirection, NoCmpMatchConfig, OnMatchCfg};

trait ComparatorChannel {}

pub struct CmpX;
impl ComparatorChannel for CmpX {}

pub struct CmpY;
impl ComparatorChannel for CmpY {}

pub trait OptionalCmp {
    type OnMatchCfg: OnMatchCfg;
    type Cfg: OptionalCmpCfg<Cmp = Self>;

    fn get_comparator_mut(&mut self) -> Option<&mut Comparator>;
}

impl OptionalCmp for Comparator {
    type OnMatchCfg = super::generator::CountingDirection;
    type Cfg = ComparatorConfig;

    fn get_comparator_mut(&mut self) -> Option<&mut Comparator> {
        Some(self)
    }
}

pub struct NoCmp;
impl OptionalCmp for NoCmp {
    type OnMatchCfg = super::generator::NoCmpMatchConfig;
    type Cfg = NoCmpCfg;

    fn get_comparator_mut(&mut self) -> Option<&mut Comparator> {
        None
    }
}

pub struct Comparator(pub(crate) mcpwm_cmpr_handle_t);

impl Comparator {
    pub(crate) unsafe fn configure(&mut self, gen: &mut mcpwm_gen_t, cfg: CountingDirection) {
        // TODO: "must be terminated by MCPWM_GEN_TIMER_EVENT_ACTION_END()"
        mcpwm_generator_set_actions_on_compare_event(
            gen,
            esp_idf_sys::mcpwm_gen_compare_event_action_t {
                direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
                comparator: self.0,
                action: cfg.counting_up.into(),
            },
        );
        mcpwm_generator_set_actions_on_compare_event(
            gen,
            esp_idf_sys::mcpwm_gen_compare_event_action_t {
                direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
                comparator: self.0,
                action: cfg.counting_down.into(),
            },
        );
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ComparatorConfig {
    flags: mcpwm_comparator_config_t__bindgen_ty_1,
}

impl Default for ComparatorConfig {
    fn default() -> Self {
        let mut flags: mcpwm_comparator_config_t__bindgen_ty_1 = Default::default();
        // TODO: What should be set here?
        flags.set_update_cmp_on_tep(1);
        flags.set_update_cmp_on_tez(1);
        flags.set_update_cmp_on_sync(1);
        Self { flags }
    }
}

pub struct NoCmpCfg;

pub trait OptionalCmpCfg {
    type OnMatchCfg: OnMatchCfg;
    type Cmp: OptionalCmp;
    unsafe fn init(self, operator_handle: mcpwm_oper_handle_t) -> Self::Cmp;
}

impl OptionalCmpCfg for NoCmpCfg {
    type OnMatchCfg = NoCmpMatchConfig;
    type Cmp = NoCmp;

    unsafe fn init(self, _operator_handle: mcpwm_oper_handle_t) -> NoCmp {
        NoCmp
    }
}
impl OptionalCmpCfg for ComparatorConfig {
    type OnMatchCfg = CountingDirection;
    type Cmp = Comparator;

    unsafe fn init(self, operator_handle: mcpwm_oper_handle_t) -> Comparator {
        let cfg = mcpwm_comparator_config_t { flags: self.flags };

        let mut cmp = ptr::null_mut();
        unsafe {
            esp!(mcpwm_new_comparator(operator_handle, &cfg, &mut cmp)).unwrap();
        }

        Comparator(cmp)
    }
}
