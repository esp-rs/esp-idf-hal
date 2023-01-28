use core::ptr;

use esp_idf_sys::{
    esp, mcpwm_cmpr_handle_t, mcpwm_comparator_config_t, mcpwm_comparator_config_t__bindgen_ty_1,
    mcpwm_gen_compare_event_action_t, mcpwm_gen_handle_t, mcpwm_gen_t, mcpwm_new_comparator,
    mcpwm_oper_handle_t, mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
    mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
};

use super::generator::CountingDirection;

trait ComparatorChannel {}

pub struct CmpX;
impl ComparatorChannel for CmpX {}

pub struct CmpY;
impl ComparatorChannel for CmpY {}

pub struct Comparator(pub(crate) mcpwm_cmpr_handle_t);

impl Comparator {
    pub(crate) unsafe fn configure(&mut self, gen: &mut mcpwm_gen_t, cfg: CountingDirection) {
        extern "C" {
            fn mcpwm_generator_set_actions_on_compare_event(
                gen: mcpwm_gen_handle_t,
                ev_act0: mcpwm_gen_compare_event_action_t,
                ev_act1: mcpwm_gen_compare_event_action_t,
                ev_act_end: mcpwm_gen_compare_event_action_t,
            ) -> esp_idf_sys::esp_err_t;
        }

        esp!(mcpwm_generator_set_actions_on_compare_event(
            gen,
            mcpwm_gen_compare_event_action_t {
                direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
                comparator: self.0,
                action: cfg.counting_up.into(),
            },
            mcpwm_gen_compare_event_action_t {
                direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_DOWN,
                comparator: self.0,
                action: cfg.counting_down.into(),
            },
            mcpwm_gen_compare_event_action_t {
                // <-- This marks the last argument in the variadic list
                comparator: ptr::null_mut(),
                ..Default::default()
            }
        ))
        .unwrap();
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ComparatorConfig {
    flags: mcpwm_comparator_config_t__bindgen_ty_1,
}

impl ComparatorConfig {
    pub(crate) unsafe fn init(self, operator_handle: mcpwm_oper_handle_t) -> Comparator {
        let cfg = mcpwm_comparator_config_t { flags: self.flags };

        let mut cmp = ptr::null_mut();
        unsafe {
            esp!(mcpwm_new_comparator(operator_handle, &cfg, &mut cmp)).unwrap();
        }

        Comparator(cmp)
    }
}

impl Default for ComparatorConfig {
    fn default() -> Self {
        let mut flags: mcpwm_comparator_config_t__bindgen_ty_1 = Default::default();
        // TODO: What should be set here?
        flags.set_update_cmp_on_tep(0);
        flags.set_update_cmp_on_tez(1);
        flags.set_update_cmp_on_sync(0);
        Self { flags }
    }
}
