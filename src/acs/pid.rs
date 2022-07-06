//限幅

#[derive(Debug)]
struct Limit {
    min: f32,
    max: f32,
}

impl Default for Limit {
    fn default() -> Self {
        Limit {
            min: -100.,
            max: 100.,
        }
    }
}

//误差
#[derive(Debug, Default)]
struct Error {
    prev: f32,
    current: f32,
    last: f32,
}

#[derive(Debug, Default)]
pub enum Mode {
    #[default]
    Place, //位置式PID
    Increasing, //增量式PID
}

pub struct Pid<Mode> {
    kp: f32,
    ki: f32,
    kd: f32,
    mode: Mode,
    setpoint: f32,       //目标值
    limit_output: Limit, //输出限幅
    limit_p: Limit,      //比例输出限幅
    limit_i: Limit,      //积分输出限幅
    limit_d: Limit,      //微分输出限幅
    error: Error,        //误差
    derror: Error,       //微分项
}

impl Pid<Mode> {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            mode: Default::default(),
            setpoint: Default::default(),
            limit_output: Default::default(),
            limit_p: Default::default(),
            limit_i: Default::default(),
            limit_d: Default::default(),
            error: Default::default(),
            derror: Default::default(),
        }
    }
    pub fn with_mode(mut self, mode: Mode) -> Self {
        self.mode = mode;
        self
    }
    pub fn with_limit_output(mut self, limit: Limit) -> Self {
        self.limit_output = limit;
        self
    }
}
