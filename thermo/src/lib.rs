#![doc = include_str!("../README.md")]

/// The target temperature of Zone 1
pub const ADDR_SET_Z1_C: u16 = 0x0001;
/// The target temperature of Zone 2
pub const ADDR_SET_Z2_C: u16 = 0x0002;
/// The target temperature of Zone 3
pub const ADDR_SET_Z3_C: u16 = 0x0003;
/// The target temperature of Zone 4
pub const ADDR_SET_Z4_C: u16 = 0x0004;
/// The target temperature of Zone 5
pub const ADDR_SET_Z5_C: u16 = 0x0005;
/// The target temperature of Zone 6
pub const ADDR_SET_Z6_C: u16 = 0x0006;

/// The Kp of Zone 1
pub const ADDR_P_Z1: u16 = 0x0007;
/// The Kp of Zone 2
pub const ADDR_P_Z2: u16 = 0x0008;
/// The Kp of Zone 3
pub const ADDR_P_Z3: u16 = 0x0009;
/// The Kp of Zone 4
pub const ADDR_P_Z4: u16 = 0x000A;
/// The Kp of Zone 5
pub const ADDR_P_Z5: u16 = 0x000B;
/// The Kp of Zone 6
pub const ADDR_P_Z6: u16 = 0x000C;

/// The Ki of Zone 1
pub const ADDR_I_Z1: u16 = 0x000D;
/// The Ki of Zone 2
pub const ADDR_I_Z2: u16 = 0x000E;
/// The Ki of Zone 3
pub const ADDR_I_Z3: u16 = 0x000F;
/// The Ki of Zone 4
pub const ADDR_I_Z4: u16 = 0x0010;
/// The Ki of Zone 5
pub const ADDR_I_Z5: u16 = 0x0011;
/// The Ki of Zone 6
pub const ADDR_I_Z6: u16 = 0x0012;

/// The Kd of Zone 1
pub const ADDR_D_Z1: u16 = 0x0013;
/// The Kd of Zone 2
pub const ADDR_D_Z2: u16 = 0x0014;
/// The Kd of Zone 3
pub const ADDR_D_Z3: u16 = 0x0015;
/// The Kd of Zone 4
pub const ADDR_D_Z4: u16 = 0x0016;
/// The Kd of Zone 5
pub const ADDR_D_Z5: u16 = 0x0017;
/// The Kd of Zone 6
pub const ADDR_D_Z6: u16 = 0x0018;

/// The Ki enabled range of Zone 1
pub const ADDR_I_RANGE_Z1_C: u16 = 0x0019;
/// The Ki enabled range of Zone 2
pub const ADDR_I_RANGE_Z2_C: u16 = 0x001A;
/// The Ki enabled range of Zone 3
pub const ADDR_I_RANGE_Z3_C: u16 = 0x001B;
/// The Ki enabled range of Zone 4
pub const ADDR_I_RANGE_Z4_C: u16 = 0x001C;
/// The Ki enabled range of Zone 5
pub const ADDR_I_RANGE_Z5_C: u16 = 0x001D;
/// The Ki enabled range of Zone 6
pub const ADDR_I_RANGE_Z6_C: u16 = 0x001E;

/// When not zero, loads value from Kpid registers
pub const ADDR_RELOAD_PID: u16 = 0x001F;
/// Total power consumed by heaters
pub const ADDR_THERMO_PWR_W: u16 = 0x0020;
/// When not zero, functional PID control is enabled
pub const ADDR_ENBL_FUNCIONAL_PID: u16 = 0x0021;
/// When not zero, power is controlled though the power registers and not from
/// the internal PID calculations
pub const ADDR_ENBL_PWR_OVERRIDE: u16 = 0x0022;

/// The heater power of Zone 1
pub const ADDR_Z1_PWR: u16 = 0x0023;
/// The heater power of Zone 2
pub const ADDR_Z2_PWR: u16 = 0x0024;
/// The heater power of Zone 3
pub const ADDR_Z3_PWR: u16 = 0x0025;
/// The heater power of Zone 4
pub const ADDR_Z4_PWR: u16 = 0x0026;
/// The heater power of Zone 5
pub const ADDR_Z5_PWR: u16 = 0x0027;
/// The heater power of Zone 6
pub const ADDR_Z6_PWR: u16 = 0x0028;
