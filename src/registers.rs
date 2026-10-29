//! BMM350 register addresses used by the driver.

pub struct Register;

impl Register {
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR_REG: u8 = 0x02;
    pub const PMU_CMD_AGGR_SET: u8 = 0x04;
    pub const PMU_CMD_AXIS_EN: u8 = 0x05;
    pub const PMU_CMD: u8 = 0x06;
    pub const PMU_CMD_STATUS_0: u8 = 0x07;
    pub const PMU_CMD_STATUS_1: u8 = 0x08;
    pub const MAG_X_XLSB: u8 = 0x31;
    pub const OTP_CMD: u8 = 0x50;
    pub const OTP_DATA_MSB: u8 = 0x52;
    pub const OTP_DATA_LSB: u8 = 0x53;
    pub const OTP_STATUS: u8 = 0x55;
    pub const CMD: u8 = 0x7e;

    pub const BMM350_CHIP_ID: u8 = 0x33;
    pub const CMD_SOFT_RESET: u8 = 0xb6;
}
