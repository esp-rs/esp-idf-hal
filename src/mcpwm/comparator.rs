use esp_idf_sys::{mcpwm_cmpr_handle_t, mcpwm_gen_t, mcpwm_generator_set_actions_on_compare_event, mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP, mcpwm_generator_action_t_MCPWM_GEN_ACTION_KEEP, mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN};

use super::generator::{CountingDirection, OnMatchCfg};


trait ComparatorChannel{}

pub struct CmpX;
impl ComparatorChannel for CmpX {}

pub struct CmpY;
impl ComparatorChannel for CmpY {}

pub trait OptionalCmp {
    type OnMatchCfg: OnMatchCfg;
    unsafe fn _configure(&mut self, gen: &mut mcpwm_gen_t, cfg: Self::OnMatchCfg);
}

impl OptionalCmp for Comparator {
    type OnMatchCfg = super::generator::CountingDirection;

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

    unsafe fn _configure(&mut self, gen: &mut mcpwm_gen_t, cfg: Self::OnMatchCfg) {
        // Do nothing
    }
}

pub struct Comparator(pub(crate)mcpwm_cmpr_handle_t);

#[derive(Debug, Clone, Copy, Default)]
pub struct ComparatorConfig {
    _todo: (),
}