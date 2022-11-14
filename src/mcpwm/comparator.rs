
trait ComparatorChannel{}

pub struct CmpX;
impl ComparatorChannel for CmpX {}

pub struct CmpY;
impl ComparatorChannel for CmpY {}

trait OptionalCmp {
    type OnMatchCfg: Into<super::generator::CountingDirection>;
}

impl OptionalCmp for Comparator {
    type OnMatchCfg = super::generator::CountingDirection;
}

struct NoCmp;
impl OptionalCmp for NoCmp {
    type OnMatchCfg = super::generator::NoCmpMatchConfig;
}

pub struct Comparator(pub(crate)mcpwm_cmpr_handle_t);

#[derive(Debug, Clone, Copy)]
pub struct ComparatorConfig {
    _: (),
}