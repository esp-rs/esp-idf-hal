use core::ptr;

use esp_idf_sys::{mcpwm_cmpr_handle_t, mcpwm_gen_t, mcpwm_generator_set_actions_on_compare_event, mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP, mcpwm_generator_action_t_MCPWM_GEN_ACTION_KEEP, mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN, mcpwm_comparator_config_t__bindgen_ty_1, esp, mcpwm_comparator_config_t, mcpwm_new_comparator, mcpwm_oper_handle_t};

use super::generator::{CountingDirection, OnMatchCfg, NoCmpMatchConfig};


trait ComparatorChannel{}

pub struct CmpX;
impl ComparatorChannel for CmpX {}

pub struct CmpY;
impl ComparatorChannel for CmpY {}

pub trait OptionalCmp {
    type OnMatchCfg: OnMatchCfg;
    type Cfg: OptionalCmpCfg<Cmp=Self>;
    unsafe fn _configure(&mut self, gen: &mut mcpwm_gen_t, cfg: Self::OnMatchCfg);
}

impl OptionalCmp for Comparator {
    type OnMatchCfg = super::generator::CountingDirection;
    type Cfg = ComparatorConfig;

    unsafe fn _configure(&mut self, gen: &mut mcpwm_gen_t, cfg: Self::OnMatchCfg) {
        // TODO: "must be terminated by MCPWM_GEN_TIMER_EVENT_ACTION_END()"
        mcpwm_generator_set_actions_on_compare_event(gen, esp_idf_sys::mcpwm_gen_compare_event_action_t {
            direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP, comparator: self.0, action: cfg.counting_up.into()
        });
        mcpwm_generator_set_actions_on_compare_event(gen, esp_idf_sys::mcpwm_gen_compare_event_action_t {
            direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN, comparator: self.0, action: cfg.counting_down.into()
        });
    }
}

pub struct NoCmp;
impl OptionalCmp for NoCmp {
    type OnMatchCfg = super::generator::NoCmpMatchConfig;
    type Cfg = NoCmpCfg;

    unsafe fn _configure(&mut self, gen: &mut mcpwm_gen_t, cfg: Self::OnMatchCfg) {
        // Do nothing
    }
}

pub struct Comparator(pub(crate)mcpwm_cmpr_handle_t);

#[derive(Debug, Clone, Copy)]
pub struct ComparatorConfig {
    flags: mcpwm_comparator_config_t__bindgen_ty_1,
}

impl Default for ComparatorConfig {
    fn default() -> Self {
        let mut flags: mcpwm_comparator_config_t__bindgen_ty_1 = Default::default();
        flags.set_update_cmp_on_tep(todo!());
        flags.set_update_cmp_on_tez(todo!());
        flags.set_update_cmp_on_sync(todo!());
        Self { flags }
    }
}

pub struct NoCmpCfg;

pub trait OptionalCmpCfg{
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
impl OptionalCmpCfg for ComparatorConfig{
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