pub const LED_LEVEL_STEP: f64 = 0.01;
pub const OTP_SHUTDOWN_TEMP: f64 = 60.0;
pub const OTP_HYSTERESIS_TEMP: f64 = 50.0;
pub const OTP_THERMOREGULATION_TEMP: f64 = 40.0;

pub const FAN_MINIMUM_DUTY_CYCLE: f64 = 0.15;

pub const ADC_DIVIDER: f64 = 22.1 / (82.0 + 22.1); // VIN--[ 82 ]--ADC--[22.1]--GND