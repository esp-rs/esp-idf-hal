
trait GeneratorChannel{
    const IS_A: bool;
}

pub struct GenA;
impl GeneratorChannel for GenA {
    const IS_A: bool = true;
}

pub struct GenB;
impl GeneratorChannel for GenB {
    const IS_A: bool = false;
}

pub struct Generator(pub(crate)mcpwm_gen_handle_t);

#[derive(Default, Clone)]
pub struct GeneratorConfig<G: GeneratorChannel, CMP_X, CMP_Y> {
    _channel: PhantomData<G>,
    on_matches_cmp_x: CMP_X,
    on_matches_cmp_y: CMP_Y,
    on_is_empty: CountingDirection,
    on_is_full: CountingDirection
}

impl GeneratorConfig<NoCmpMatchConfig, NoCmpMatchConfig> {
    fn empty() -> Self {
        GeneratorConfig {
            _channel: PhantomData,
            on_matches_cmp_x: NoCmpMatchConfig,
            on_matches_cmp_y: NoCmpMatchConfig,
            on_is_empty: CountingDirection { 
                counting_up: GeneratorAction::Nothing,
                counting_down: GeneratorAction::Nothing,
            },
            on_is_full: CountingDirection { 
                counting_up: GeneratorAction::Nothing,
                counting_down: GeneratorAction::Nothing,
            }
        }
    }
}

impl Into<CountingDirection> for NoCmpMatchConfig {
    fn into(self) -> CountingDirection {
        CountingDirection { 
            counting_up: GeneratorAction::Nothing,
            counting_down: GeneratorAction::Nothing,
        }
    }
}

impl<CMP_X, CMP_Y> Into<GeneratorConfig<CountingDirection, CountingDirection>> for GeneratorConfig<CMP_X, CMP_Y> 
    where
        CMP_X: Into<CountingDirection>,
        CMP_Y: Into<CountingDirection>
{
    fn into(self) -> GeneratorConfig<CountingDirection, CountingDirection> {
        GeneratorConfig {
            _channel,
            on_matches_cmp_x,
            on_matches_cmp_y,
            on_is_empty,
            on_is_full
        } = self;

        GeneratorConfig {
            _channel,
            on_matches_cmp_x: on_matches_cmp_x.into(),
            on_matches_cmp_y: on_matches_cmp_y.into(),
            on_is_empty,
            on_is_full
        }
    }

}

pub struct NoCmpMatchConfig;

// TODO: Come up with better name
#[derive(Clone, Copy)]
pub struct CountingDirection {
    counting_up: GeneratorAction,
    counting_down: GeneratorAction
}

pub enum GeneratorAction {
    Nothing,
    SetLow,
    SetHigh,
    Toggle
}

impl DutyMode {
    fn into_duty_cfg<G: Generator>(self) -> GeneratorConfig<G, CountingDirection, CountingDirection> {
        match val {
            DutyMode::ActiveHigh => {
                let mut duty_config = GeneratorConfig::empty();
                duty_config.on_is_empty.counting_up = GeneratorAction::SetHigh;
                if G::IS_A {
                    duty_config.on_matches_cmp_a.counting_up = GeneratorAction::SetLow;
                } else {
                    duty_config.on_matches_cmp_b.counting_up = GeneratorAction::SetLow;
                }
                duty_config
            },
            DutyMode::ActiveLow => {
                let mut duty_config = GeneratorConfig::empty();
                duty_config.on_is_empty.counting_up = GeneratorAction::SetLow;
                if G::IS_A {
                    duty_config.on_matches_cmp_a.counting_up = GeneratorAction::SetHigh;
                } else {
                    duty_config.on_matches_cmp_b.counting_up = GeneratorAction::SetHigh;
                }

                duty_config
            },
        }
    }
}