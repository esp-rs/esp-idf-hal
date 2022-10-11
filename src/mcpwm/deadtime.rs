// TODO: This is stolen from the MCPWM 4.4 PR, check if/how this should be changed for 5.0


/// Dead time config for MCPWM operator
///
/// `rising_edge_delay` and `falling_edge_delay` is time as in number of clock cycles after the MCPWM modules group prescaler.
///
/// Note that the dead times are calculated from MCPWMXA's flanks unless explicitly stated otherwise
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum DeadtimeConfig {
    // TODO: Figure out what all of those options do and give them nice descriptions
    /// MCPWM_BYPASS_RED
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
    ///               .                    .   .
    ///               .                    .   .
    ///               .--------------------.   .
    ///               |                    |   .
    /// MCPWMXA in    |                    |   .
    ///               |                    |   .
    /// ---------------                    ---------------------
    ///               .                    .   .
    ///               .                    .   .
    ///               .--------------------.   .
    ///               |                    |   .
    /// MCPWMXA out   |                    |   .
    ///               |                    |   .
    /// ---------------                    ---------------------
    ///               .                    .   .
    ///               .                    .   .
    ///               .------------------------.
    ///               |                   >.   |< fed
    /// MCPWMXB out   |                    .   |
    ///               |                    .   |
    /// --------------.                    .   -----------------
    ///               .                    .   .
    ///               .                    .   .
    /// ```
    BypassRisingEdge { falling_edge_delay: u16 },

    /// MCPWM_BYPASS_FED
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
    ///               .   .                .
    ///               .   .                .
    ///               .--------------------.
    ///               |   .                |
    /// MCPWMXA in    |   .                |
    ///               |   .                |
    /// ---------------   .                ---------------------
    ///               .   .                .
    ///               .   .                .
    ///               .   .----------------.
    ///          red >.   |<               |
    /// MCPWMXA out   .   |                |
    ///               .   |                |
    /// -------------------                ---------------------
    ///               .   .                .
    ///               .   .                .
    ///               .--------------------.
    ///               |   .                |
    /// MCPWMXB out   |   .                |
    ///               |   .                |
    /// ---------------   .                ---------------------
    ///               .   .                .
    ///               .   .                .
    /// ```
    BypassFallingEdge { rising_edge_delay: u16 },

    /// MCPWM_ACTIVE_HIGH_MODE
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXA in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .----------------.   .
    ///          red >.   |<               |   .
    /// MCPWMXA out   .   |                |   .
    ///               .   |                |   .
    /// -------------------                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .------------------------.
    ///               |   .               >.   |< fed
    /// MCPWMXB out   |   .                .   |
    ///               |   .                .   |
    /// --------------.   .                .   -----------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ```
    ActiveHigh {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },

    /// MCPWM_ACTIVE_LOW_MODE
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXA in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ------------------.                .--------------------
    ///          red >.   |<               |   .
    /// MCPWMXA out   .   |                |   .
    ///               .   |                |   .
    ///               .   ------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// --------------.   .                .   .----------------
    ///               |   .               >.   |< fed
    /// MCPWMXB out   |   .                .   |
    ///               |   .                .   |
    ///               --------------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ```
    ActiveLow {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },

    /// MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXA in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .----------------.   .
    ///          red >.   |<               |   .
    /// MCPWMXA out   .   |                |   .
    ///               .   |                |   .
    /// -------------------                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// --------------.   .                .   .----------------
    ///               |   .               >.   |< fed
    /// MCPWMXB out   |   .                .   |
    ///               |   .                .   |
    ///               --------------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ```
    ActiveHighComplement {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },

    /// MCPWM_ACTIVE_LOW_COMPLIMENT_MODE
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXA in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ------------------.                .--------------------
    ///          red >.   |<               |   .
    /// MCPWMXA out   .   |                |   .
    ///               .   |                |   .
    ///               .   ------------------   .
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .------------------------.
    ///               |   .               >.   |< fed
    /// MCPWMXB out   |   .                .   |
    ///               |   .                .   |
    /// ---------------   .                .   -----------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ```
    ActiveLowComplement {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },

    /// MCPWM_ACTIVE_RED_FED_FROM_PWMXA
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXA in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .--------------------.
    ///          red >.   |<               .   |
    /// MCPWMXA out   .   |                .   |
    ///               .   |                .   |
    /// -------------------                .   ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .--------------------.
    ///          red >.   |<               .   |
    /// MCPWMXB out   .   |                .   |
    ///               .   |                .   |
    /// -------------------                .   ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ```
    ActiveRedFedFromPwmxa {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },

    /// MCPWM_ACTIVE_RED_FED_FROM_PWMXB
    ///
    /// Note that `MCPWMXA in` will be completely ignored. This means `Operator::set_duty_a` will
    /// have no effect with this dead time mode
    /// ```
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXB in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .--------------------.
    ///          red >.   |<               .   |
    /// MCPWMXA out   .   |                .   |
    ///               .   |                .   |
    /// -------------------                .   ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .--------------------.
    ///          red >.   |<               .   |
    /// MCPWMXB out   .   |                .   |
    ///               .   |                .   |
    /// -------------------                .   ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ```
    ActiveRedFedFromPwmxb {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },
}

impl DeadtimeConfig {
    fn as_args(&self) -> DeadtimeArgs {
        match *self {
            DeadtimeConfig::BypassRisingEdge { falling_edge_delay } => DeadtimeArgs {
                rising_edge_delay: 0,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_BYPASS_RED,
            },

            DeadtimeConfig::BypassFallingEdge { rising_edge_delay } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay: 0,
                mode: mcpwm_deadtime_type_t_MCPWM_BYPASS_FED,
            },

            DeadtimeConfig::ActiveHigh {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_HIGH_MODE,
            },

            DeadtimeConfig::ActiveLow {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_LOW_MODE,
            },

            DeadtimeConfig::ActiveHighComplement {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
            },

            DeadtimeConfig::ActiveLowComplement {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_LOW_COMPLIMENT_MODE,
            },

            DeadtimeConfig::ActiveRedFedFromPwmxa {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_RED_FED_FROM_PWMXA,
            },

            DeadtimeConfig::ActiveRedFedFromPwmxb {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_RED_FED_FROM_PWMXB,
            },
        }
    }
}

struct DeadtimeArgs {
    rising_edge_delay: u16,
    falling_edge_delay: u16,
    mode: mcpwm_deadtime_type_t,
}