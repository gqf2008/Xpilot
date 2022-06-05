use core::fmt::Formatter;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use libm::*;
pub struct Mpu6050<I2c>
where
    I2c: Write + WriteRead,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    i2c: I2c,
    address: u8,
    acc_range: AccelRange,
    gyro_range: GyroRange,
    dlpf: DLPF,
    interrupt: bool,
    mean_imu_raw_data: RawData,
    gyro_offset: Gyro,
    mean_filter_fifo: [[i16; 15]; 6],
}

impl<I2c> Mpu6050<I2c>
where
    I2c: Write + WriteRead,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    pub fn new(i2c: I2c) -> Self {
        Self {
            i2c,
            address: 0x68,
            acc_range: AccelRange::_4G,
            gyro_range: GyroRange::_2000DEGS,
            dlpf: DLPF::_94HZ,
            interrupt: false,
            mean_imu_raw_data: Default::default(),
            gyro_offset: Default::default(),
            mean_filter_fifo: Default::default(),
        }
    }
    pub fn with_address(mut self, address: u8) -> Self {
        self.address = address;
        self
    }
    pub fn with_acc_range(mut self, acc_range: AccelRange) -> Self {
        self.acc_range = acc_range;
        self
    }
    pub fn with_gyro_range(mut self, gyro_range: GyroRange) -> Self {
        self.gyro_range = gyro_range;
        self
    }
    pub fn with_dlpf(mut self, dlpf: DLPF) -> Self {
        self.dlpf = dlpf;
        self
    }
    pub fn with_interrupt(mut self, interrupt: bool) -> Self {
        self.interrupt = interrupt;
        self
    }

    pub fn build(mut self) -> Result<Self, Error<I2c>> {
        self.who_am_i()?;
        self.disable_sleep()?;
        self.set_dlpf(self.dlpf)?;
        self.set_gyro_range(self.gyro_range)?;
        self.set_accel_range(self.acc_range)?;
        if self.interrupt {
            self.set_interrupt_pin_high()?;
            self.enable_data_interrupt()?;
        }
        self.set_sample_rate_divider(0x00)?; //100hz
        xtask::delay_us(100000);
        Ok(self)
    }
}

impl<I2c> Mpu6050<I2c>
where
    I2c: Write + WriteRead,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    pub fn who_am_i(&mut self) -> Result<(), Error<I2c>> {
        let id = self.read_register(Register::WhoAmI)?;
        if self.address != id {
            Err(Error::WrongDevice)
        } else {
            Ok(())
        }
    }
    // 解除休眠
    pub fn disable_sleep(&mut self) -> Result<(), Error<I2c>> {
        self.write_register(Register::PwrMgmt1, 0x01)
    }
    // 设置低通滤波
    pub fn set_dlpf(&mut self, dlpf: DLPF) -> Result<(), Error<I2c>> {
        self.write_register(Register::Config, dlpf as u8)
    }

    // 设置加速度计测量范围
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Error<I2c>> {
        self.write_register(Register::AccelConfig, range as u8)
    }
    // 设置陀螺仪测量范围
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Error<I2c>> {
        self.write_register(Register::GyroConfig, range as u8)
    }

    // 配置中断引脚为高电平
    pub fn set_interrupt_pin_high(&mut self) -> Result<(), Error<I2c>> {
        self.write_register(Register::InterruptPinConfig, 0x02)
    }
    // 使能数据中断
    pub fn enable_data_interrupt(&mut self) -> Result<(), Error<I2c>> {
        self.write_register(Register::InterruptEnable, 0x01)
    }

    // 设置采用率
    pub fn set_sample_rate_divider(&mut self, div: u8) -> Result<(), Error<I2c>> {
        self.write_register(Register::SmpRtDiv, div)
    }

    // pub fn accel(&mut self) -> Result<Accel, Error<I2c>> {
    //     let mut data = [0; 6];
    //     self.read_registers(Register::AccelX_H, &mut data)?;
    //     Ok(Accel::new(data))
    // }

    // pub fn gyro(&mut self) -> Result<Gyro, Error<I2c>> {
    //     let mut data = [0; 6];
    //     self.read_registers(Register::GyroX_H, &mut data)?;
    //     Ok(Gyro::new(data))
    // }

    pub fn accel_gyro(&mut self) -> Result<ImuData, Error<I2c>> {
        let mut data = [0; 14];
        self.read_registers(Register::AccelX_H, &mut data)?;
        let raw = RawData::new(data);
        self.mean_filter(&raw);
        let mut imu_data = ImuData::default();

        // 转为g
        let g = 32768.0 / self.acc_range.range() as f32;
        imu_data.accel.x = self.mean_imu_raw_data.ax as f32 / g;
        imu_data.accel.y = self.mean_imu_raw_data.ay as f32 / g;
        imu_data.accel.z = self.mean_imu_raw_data.az as f32 / g;

        // 转成度每秒
        let a = 32768.0 / self.gyro_range.range() as f32;
        imu_data.gyro.x = self.mean_imu_raw_data.gx as f32 / a - self.gyro_offset.x;
        imu_data.gyro.y = self.mean_imu_raw_data.gy as f32 / a - self.gyro_offset.y;
        imu_data.gyro.z = self.mean_imu_raw_data.gz as f32 / a - self.gyro_offset.z;
        imu_data.temp = 36.53 + self.mean_imu_raw_data.temp as f32 / 340.0;
        Ok(imu_data)
    }

    //[0]-[9]为最近10次数据 [10]为10次数据的平均值
    fn mean_filter(&mut self, raw: &RawData) {
        const MEAN_FILTER_ROWS: usize = 6;
        const MEAN_FILTER_COLS: usize = 8;

        for row in 0..MEAN_FILTER_ROWS {
            for col in 1..MEAN_FILTER_COLS {
                self.mean_filter_fifo[row][col - 1] = self.mean_filter_fifo[row][col];
            }
        }

        self.mean_filter_fifo[0][MEAN_FILTER_COLS - 1] = raw.ax;
        self.mean_filter_fifo[1][MEAN_FILTER_COLS - 1] = raw.ay;
        self.mean_filter_fifo[2][MEAN_FILTER_COLS - 1] = raw.az;
        self.mean_filter_fifo[3][MEAN_FILTER_COLS - 1] = raw.gx;
        self.mean_filter_fifo[4][MEAN_FILTER_COLS - 1] = raw.gy;
        self.mean_filter_fifo[5][MEAN_FILTER_COLS - 1] = raw.gz;

        for row in 0..MEAN_FILTER_ROWS {
            let mut sum = 0i32;
            for col in 0..MEAN_FILTER_COLS {
                sum += self.mean_filter_fifo[row][col] as i32;
            }
            self.mean_filter_fifo[row][MEAN_FILTER_COLS] = (sum / MEAN_FILTER_COLS as i32) as i16;
        }

        self.mean_imu_raw_data.ax = self.mean_filter_fifo[0][MEAN_FILTER_COLS];
        self.mean_imu_raw_data.ay = self.mean_filter_fifo[1][MEAN_FILTER_COLS];
        self.mean_imu_raw_data.az = self.mean_filter_fifo[2][MEAN_FILTER_COLS];
        self.mean_imu_raw_data.gx = self.mean_filter_fifo[3][MEAN_FILTER_COLS];
        self.mean_imu_raw_data.gy = self.mean_filter_fifo[4][MEAN_FILTER_COLS];
        self.mean_imu_raw_data.gz = self.mean_filter_fifo[5][MEAN_FILTER_COLS];
    }

    pub fn cal_gyro_offset(&mut self) -> Result<(), Error<I2c>> {
        const IMU_INIT_GYRO_THRESHOLD: f32 = 4.0;
        const IMU_STATIC_GYRO_THRESHOLD: f32 = 5.0;
        let mut gyro_sum = Gyro::default();
        let mut count = 300;
        loop {
            let offset_data = self.accel_gyro()?;
            if fabsf(offset_data.gyro.x) < IMU_INIT_GYRO_THRESHOLD
                && fabsf(offset_data.gyro.y) < IMU_INIT_GYRO_THRESHOLD
                && fabsf(offset_data.gyro.z) < IMU_INIT_GYRO_THRESHOLD
            {
                count -= 1;
                gyro_sum.x += offset_data.gyro.x;
                gyro_sum.y += offset_data.gyro.y;
                gyro_sum.z += offset_data.gyro.z;
                if count == 0 {
                    break;
                }
                xtask::delay_us(10000);
            }
        }
        let count = count as f32;
        self.gyro_offset.x = gyro_sum.x / count;
        self.gyro_offset.y = gyro_sum.y / count;
        self.gyro_offset.z = gyro_sum.z / count;
        log::info!(
            "IMU: calibrate offset seccess, gx: {} gy: {} gz: {}",
            self.gyro_offset.x,
            self.gyro_offset.y,
            self.gyro_offset.z
        );

        Ok(())
    }
}

impl<I2c> Mpu6050<I2c>
where
    I2c: Write + WriteRead,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    pub(crate) fn read(&mut self, bytes: &[u8], response: &mut [u8]) -> Result<(), Error<I2c>> {
        self.i2c
            .write_read(self.address, bytes, response)
            .map_err(|e| Error::WriteReadError(e))
    }

    pub(crate) fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2c>> {
        self.i2c
            .write(self.address, bytes)
            .map_err(|e| Error::WriteError(e))
    }

    pub(crate) fn read_register(&mut self, reg: Register) -> Result<u8, Error<I2c>> {
        let mut buf = [0; 1];
        self.read(&[reg as u8], &mut buf)?;
        Ok(buf[0])
    }

    pub(crate) fn read_registers<'a>(
        &mut self,
        reg: Register,
        buf: &'a mut [u8],
    ) -> Result<&'a [u8], Error<I2c>> {
        self.read(&[reg as u8], buf)?;
        Ok(buf)
    }

    pub(crate) fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<I2c>> {
        self.write(&[reg as u8, value])
    }
}

/// Error for sensor operations.
pub enum Error<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    WriteError(<I2c as Write>::Error),
    WriteReadError(<I2c as WriteRead>::Error),
    WrongDevice,
}

impl<I2c> core::fmt::Debug for Error<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::result::Result<(), core::fmt::Error> {
        match self {
            Error::WriteReadError(e) => f.debug_tuple("WriteReadError").field(e).finish(),
            Error::WriteError(e) => f.debug_tuple("WriteError").field(e).finish(),
            Error::WrongDevice => f.write_str("WrongDevice"),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum GyroRange {
    _250DEGS = 0x00,
    _500DEGS = 0x08,
    _1000DEGS = 0x10,
    _2000DEGS = 0x18,
}

impl GyroRange {
    pub fn range(self) -> u16 {
        match self {
            _250DEGS => 250,
            _500DEGS => 500,
            _1000DEGS => 1000,
            _2000DEGS => 2000,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum AccelRange {
    _2G = 0x00,
    _4G = 0x08,
    _8G = 0x10,
    _16G = 0x18,
}

impl AccelRange {
    pub fn range(self) -> u16 {
        match self {
            _2G => 2,
            _4G => 4,
            _8G => 6,
            _16G => 16,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum DLPF {
    _260HZ = 0x00, // acc delay 0ms gyro 0.98ms
    _184HZ = 0x01, // acc delay 2ms gyro 1.9ms
    _94HZ = 0x02,  // acc delay 3ms gyro 2.8ms
    _44HZ = 0x03,  // acc delay 4.9ms gyro 4.8ms
}

#[derive(Copy, Clone, Debug, Default)]
pub struct EulerAngle {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

#[derive(Debug, Default)]
struct RawData {
    pub ax: i16,
    pub ay: i16,
    pub az: i16,
    pub temp: i16,
    pub gx: i16,
    pub gy: i16,
    pub gz: i16,
}

impl RawData {
    pub(crate) fn new(data: [u8; 14]) -> Self {
        Self {
            ax: i16::from_be_bytes([data[0], data[1]]),
            ay: i16::from_be_bytes([data[2], data[3]]),
            az: i16::from_be_bytes([data[4], data[5]]),
            temp: i16::from_be_bytes([data[6], data[7]]),
            gx: i16::from_be_bytes([data[8], data[9]]),
            gy: i16::from_be_bytes([data[10], data[11]]),
            gz: i16::from_be_bytes([data[12], data[13]]),
        }
    }
}

#[derive(Debug, Default)]
pub struct ImuData {
    pub accel: Accel,
    pub temp: f32,
    pub gyro: Gyro,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Accel {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Gyro {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
pub enum Register {
    Config = 0x1A,
    PwrMgmt1 = 0x6B,
    SmpRtDiv = 0x19,
    WhoAmI = 0x75,
    AccelOffsetX_H = 0x06,
    AccelOffsetX_L = 0x07,
    AccelOffsetY_H = 0x08,
    AccelOffsetY_L = 0x09,
    AccelOffsetZ_H = 0x0A,
    AccelOffsetZ_L = 0x0B,

    GyroOffsetX_H = 0x13,
    GyroOffsetX_L = 0x14,
    GyroOffsetY_H = 0x15,
    GyroOffsetY_L = 0x16,
    GyroOffsetZ_H = 0x17,
    GyroOffsetZ_L = 0x18,

    AccelX_H = 0x3B,
    AccelX_L = 0x3C,
    AccelY_H = 0x3D,
    AccelY_L = 0x3E,
    AccelZ_H = 0x3F,
    AccelZ_L = 0x40,

    AccelConfig = 0x1C,

    GyroX_H = 0x43,
    GyroX_L = 0x44,
    GyroY_H = 0x45,
    GyroY_L = 0x46,
    GyroZ_H = 0x47,
    GyroZ_L = 0x48,

    GyroConfig = 0x1B,

    UserCtrl = 0x6A,

    FifoEn = 0x23,
    FifoCount_H = 0x72,
    FifoCount_L = 0x73,
    FifoRw = 0x74,

    // ---
    BankSel = 0x6D,
    MemStartAddr = 0x6E,
    MemRw = 0x6F,
    PrgmStart = 0x70,
    DmpConfig = 0x71,

    InterruptPinConfig = 0x37, //Interrupt pin configuration register
    InterruptEnable = 0x38,    // Interrupt enable configuration register
    InterruptStatus = 0x3A,    // Interrupt status register
}
