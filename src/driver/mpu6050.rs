use crate::driver::{Accel, Gyro, ImuData, Quaternion};
use core::fmt::Formatter;
use embedded_hal::blocking::i2c::{Write, WriteRead};

pub const SAMPLE_RATE: u16 = 200;
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
    dmp: bool,
    offset: RawData,
    sample_rate: u16,
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
            acc_range: AccelRange::_2G,
            gyro_range: GyroRange::_2000DEGS,
            dlpf: DLPF::_5_5HZ,
            interrupt: false,
            dmp: false,
            offset: Default::default(),
            sample_rate: SAMPLE_RATE,
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
    pub fn with_interrupt(mut self) -> Self {
        self.interrupt = true;
        self
    }

    pub fn with_dmp(mut self) -> Self {
        self.dmp = true;
        self
    }

    // pub fn with_sample_rate(mut self, rate: u16) -> Self {
    //     self.sample_rate = rate;
    //     self
    // }

    pub fn build(mut self) -> Result<Self, Error<I2c>> {
        log::info!(
            "Address: 0x{:02X} dlpf: {:?} acc_range: {:?} gyro_range: {:?} sample_rate: {}Hz",
            self.address,
            self.dlpf,
            self.acc_range,
            self.gyro_range,
            self.sample_rate,
        );
        self.who_am_i()?;
        self.reset()?;
        self.disable_sleep()?;
        self.set_dlpf(self.dlpf)?;
        self.set_gyro_range(self.gyro_range)?;
        self.set_accel_range(self.acc_range)?;
        if self.interrupt {
            self.set_interrupt_pin_high()?;
            self.enable_data_interrupt()?;
        }
        self.set_sample_rate(self.sample_rate)?;
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
    // 检查设备地址是否正确
    pub fn who_am_i(&mut self) -> Result<(), Error<I2c>> {
        let id = self.read_register(Register::WhoAmI)?;
        if self.address != id {
            Err(Error::WrongDevice)
        } else {
            Ok(())
        }
    }
    pub fn reset(&mut self) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::PwrMgmt1)?;
        value |= 1 << 7;
        self.write_register(Register::PwrMgmt1, value)?;
        xtask::delay_us(200_000);
        Ok(())
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

    // 设置采样频率，单位HZ，范围[4..1000]
    //采样频率=陀螺仪输出频率/(1 + SMPLRT_DIV)
    //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    //SMPLRT_DIV= Gyroscope Output Rate/Sample Rate -1
    pub fn set_sample_rate(&mut self, rate: u16) -> Result<(), Error<I2c>> {
        if rate < 4 || rate > 1000 {
            return Err(Error::IllegalParameter);
        }
        self.write_register(Register::SmpRtDiv, (1000 / rate) as u8 - 1)
    }

    // 读取温度
    pub fn temp(&mut self) -> Result<f32, Error<I2c>> {
        Ok(self.accel_gyro()?.temp.unwrap_or_default())
    }

    // 读取重力加速度
    pub fn accel(&mut self) -> Result<Accel, Error<I2c>> {
        Ok(self.accel_gyro()?.accel.unwrap_or_default())
    }

    //读取角速度
    pub fn gyro(&mut self) -> Result<Gyro, Error<I2c>> {
        Ok(self.accel_gyro()?.gyro.unwrap_or_default())
    }

    // 读取imu数据
    pub fn accel_gyro(&mut self) -> Result<ImuData, Error<I2c>> {
        Ok(self
            .raw_accel_gyro()?
            .calibrate(&self.offset)
            .to_imu_data(self.acc_range.range(), self.gyro_range.range()))
    }

    // 读取原始数据
    pub fn raw_accel_gyro(&mut self) -> Result<RawData, Error<I2c>> {
        let mut data = [0; 14];
        self.read_registers(Register::AccelX_H, &mut data)?;
        let data = RawData::new(data);
        Ok(data)
    }

    // 统计平均求偏移量
    pub fn cal_gyro_offset(&mut self) -> Result<(), Error<I2c>> {
        log::debug!("IMU: calibrate offset");
        let count = 300;
        for _ in 0..count {
            let raw = self.raw_accel_gyro()?;
            self.offset.ax += raw.ax;
            self.offset.ay += raw.ay;
            self.offset.az += raw.az;
            self.offset.gx += raw.gx;
            self.offset.gy += raw.gy;
            self.offset.gz += raw.gz;
            xtask::delay_us(10000);
        }

        self.offset.ax /= count;
        self.offset.ay /= count;
        self.offset.az /= count;
        // self.offset.az += 16384; //设芯片Z轴竖直向下，设定静态工作点。
        self.offset.az /= count;
        self.offset.gx /= count;
        self.offset.gy /= count;
        self.offset.gz /= count;

        log::info!(
            "IMU: calibrate offset ax:{} ay:{} az:{} gx: {} gy: {} gz: {}",
            self.offset.ax,
            self.offset.ay,
            self.offset.az,
            self.offset.gx,
            self.offset.gy,
            self.offset.gz
        );

        Ok(())
    }

    pub fn update_quaternion(&self, imu: &mut ImuData) {
        unsafe {
            use libm::*;
            #[allow(non_upper_case_globals)]
            const Kp: f32 = 1.0; // 比例增益支配率收敛到加速度计/磁强计
            #[allow(non_upper_case_globals)]
            const Ki: f32 = 0.01; //0.002; // 积分增益支配率的陀螺仪偏见的衔接

            const HALF_T: f32 = 1.0 / SAMPLE_RATE as f32 / 2.0; // 采样周期的一半
            #[allow(non_upper_case_globals)]
            static mut q0: f32 = 1.0; // 四元数的元素，代表估计方向
            #[allow(non_upper_case_globals)]
            static mut q1: f32 = 0.0;
            #[allow(non_upper_case_globals)]
            static mut q2: f32 = 0.0;
            #[allow(non_upper_case_globals)]
            static mut q3: f32 = 0.0;

            // 按比例缩小积分误差
            #[allow(non_upper_case_globals)]
            static mut ex_int: f32 = 0.0;
            #[allow(non_upper_case_globals)]
            static mut ey_int: f32 = 0.0;
            #[allow(non_upper_case_globals)]
            static mut ez_int: f32 = 0.0;
            let mut ax = if let Some(accel) = imu.accel {
                accel.x
            } else {
                0.0
            };
            let mut ay = if let Some(accel) = imu.accel {
                accel.y
            } else {
                0.0
            };
            let mut az = if let Some(accel) = imu.accel {
                accel.z
            } else {
                0.0
            };
            let mut gx = if let Some(gyro) = imu.gyro {
                gyro.x
            } else {
                0.0
            };
            let mut gy = if let Some(gyro) = imu.gyro {
                gyro.y
            } else {
                0.0
            };
            let mut gz = if let Some(gyro) = imu.gyro {
                gyro.z
            } else {
                0.0
            };
            // 测量正常化
            let mut norm = sqrtf(ax * ax + ay * ay + az * az);
            ax = ax / norm; //单位化
            ay = ay / norm;
            az = az / norm;

            // 估计方向的重力
            let vx = 2.0 * (q1 * q3 - q0 * q2);
            let vy = 2.0 * (q0 * q1 + q2 * q3);
            let vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

            // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
            let ex = ay * vz - az * vy;
            let ey = az * vx - ax * vz;
            let ez = ax * vy - ay * vx;

            // 积分误差比例积分增益
            ex_int = ex_int + ex * Ki;
            ey_int = ey_int + ey * Ki;
            ez_int = ez_int + ez * Ki;

            // 调整后的陀螺仪测量
            gx = gx + Kp * ex + ex_int;
            gy = gy + Kp * ey + ey_int;
            gz = gz + Kp * ez + ez_int;

            // 整合四元数率和正常化
            q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * HALF_T;
            q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * HALF_T;
            q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * HALF_T;
            q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * HALF_T;

            // 正常化四元，求平方根
            norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
            q0 = q0 / norm;
            q1 = q1 / norm;
            q2 = q2 / norm;
            q3 = q3 / norm;
            imu.quaternion = Some(Quaternion {
                w: q0,
                x: q1,
                y: q2,
                z: q3,
            });
        }
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
    IllegalParameter,
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
            Error::IllegalParameter => f.write_str("IllegalParameter"),
        }
    }
}

//Register 27 – Gyroscope Configuration
//Register(Hex) Register(Decimal) Bit7  Bit6  Bit5  Bit4   Bit3 Bit2 Bit1 Bit0
//   1B              27           XG_ST YG_ST ZG_ST FS_SEL[1:0]   -   -    -

#[derive(Copy, Clone, Debug)]
pub enum GyroRange {
    _250DEGS = 0b00000000,
    _500DEGS = 0b00001000,
    _1000DEGS = 0b00010000,
    _2000DEGS = 0b00011000,
}

impl GyroRange {
    pub fn range(self) -> f32 {
        match self {
            _250DEGS => 131.0,
            _500DEGS => 65.5,
            _1000DEGS => 32.8,
            _2000DEGS => 16.4,
        }
    }
}

//Register 28 – Accelerometer Configuration
// Register(Hex) Register(Decimal) Bit7  Bit6  Bit5  Bit4    Bit3    Bit2 Bit1 Bit0
//    1C                28         XA_ST YA_ST ZA_ST AFS_SEL[1:0]   -   -   -
#[derive(Copy, Clone, Debug)]
pub enum AccelRange {
    _2G = 0b00000000,
    _4G = 0b00001000,
    _8G = 0b00010000,
    _16G = 0b00011000,
}

impl AccelRange {
    pub fn range(self) -> f32 {
        match self {
            _2G => 16384.0,
            _4G => 8192.0,
            _8G => 4096.0,
            _16G => 2048.0,
        }
    }
}

//Register 26 – Configuration
//Register(Hex) Register(Decimal) Bit7 Bit6 Bit5  Bit4  Bit3  Bit2 Bit1 Bit0
//     1A           26             -    -   EXT_SYNC_SET[2:0] DLPF_CFG[2:0]
#[derive(Copy, Clone, Debug)]
pub enum DLPF {
    //_260_256HZ = 0b00000000, //Fs=8kHZ
    _184_188HZ = 0b00000001, //Fs=1kHZ
    _94_98HZ = 0b00000010,   //Fs=1kHZ
    _44_42HZ = 0b00000011,   //Fs=1kHZ
    _21_20HZ = 0b00000100,   //Fs=1kHZ
    _10_10HZ = 0b00000101,   //Fs=1kHZ
    _5_5HZ = 0b00000110,     //Fs=1kHZ
                             //_Off = 0b00000111, //Fs=8kHZ
}

#[derive(Debug, Default, Clone, Copy)]
pub struct RawData {
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

    pub(crate) fn calibrate(mut self, offset: &Self) -> Self {
        self.ax -= offset.ax;
        self.ay -= offset.ay;
        self.az -= offset.az;
        self.gx -= offset.gx;
        self.gy -= offset.gy;
        self.gz -= offset.gz;
        self
    }

    pub(crate) fn to_imu_data(self, acc_range: f32, gyro_range: f32) -> ImuData {
        const PI_180: f32 = core::f32::consts::PI / 180.0;
        ImuData {
            accel: Some(Accel {
                x: self.ax as f32 / acc_range,
                y: self.ay as f32 / acc_range,
                z: self.az as f32 / acc_range,
            }),
            temp: Some(36.53 + self.temp as f32 / 340.0),
            gyro: Some(Gyro {
                x: self.gx as f32 * PI_180 / gyro_range,
                y: self.gy as f32 * PI_180 / gyro_range,
                z: self.gz as f32 * PI_180 / gyro_range,
            }),
            compass: Default::default(),
            quaternion: Default::default(),
        }
    }
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
