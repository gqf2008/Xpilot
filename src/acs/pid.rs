use num_traits::float::FloatCore;

//限幅
#[derive(Debug)]
pub struct Limit<T> {
    min: T,
    max: T,
}

impl<T: FloatCore> Default for Limit<T> {
    fn default() -> Self {
        Limit {
            min: T::min_value(),
            max: T::max_value(),
        }
    }
}

//误差
#[derive(Debug)]
struct Error<T> {
    prev: T,    //上一次值
    last: T,    //最后一次值
    current: T, //当前值
}

impl<T: FloatCore> Default for Error<T> {
    fn default() -> Self {
        Self {
            prev: T::zero(),
            last: T::zero(),
            current: T::zero(),
        }
    }
}

#[derive(Debug, Default)]
pub enum Mode {
    #[default]
    Position, //位置式PID
    Increasing, //增量式PID
}

#[derive(Debug)]
pub struct Pid<T: FloatCore> {
    kp: T,
    ki: T,
    kd: T,
    mode: Mode,
    setpoint: T,                 //目标值
    limit_out: Option<Limit<T>>, //输出限幅
    limit_p: Option<Limit<T>>,   //比例输出限幅
    limit_i: Option<Limit<T>>,   //积分输出限幅
    limit_d: Option<Limit<T>>,   //微分输出限幅
    error: Error<T>,             //误差
    derror: Error<T>,            //微分误差
    p_out: T,
    i_out: T,
    d_out: T,
    out: T,
}

impl<T: FloatCore> Pid<T> {
    pub fn new(kp: T, ki: T, kd: T) -> Self {
        Self {
            kp,
            ki,
            kd,
            mode: Default::default(),
            setpoint: T::zero(),
            limit_out: Default::default(),
            limit_p: Default::default(),
            limit_i: Default::default(),
            limit_d: Default::default(),
            error: Error::default(),
            derror: Default::default(),
            p_out: T::zero(),
            i_out: T::zero(),
            d_out: T::zero(),
            out: T::zero(),
        }
    }
    pub fn with_mode(mut self, mode: Mode) -> Self {
        self.mode = mode;
        self
    }
    pub fn with_limit_out(mut self, limit: Limit<T>) -> Self {
        self.limit_out = Some(limit);
        self
    }
    pub fn with_limit_proportion(mut self, limit: Limit<T>) -> Self {
        self.limit_d = Some(limit);
        self
    }
    pub fn with_limit_integral(mut self, limit: Limit<T>) -> Self {
        self.limit_i = Some(limit);
        self
    }
    pub fn with_limit_differential(mut self, limit: Limit<T>) -> Self {
        self.limit_d = Some(limit);
        self
    }
}

impl<T: FloatCore> Pid<T> {
    //设置目标值
    pub fn setpoint(&mut self, setpoint: T) {
        self.setpoint = setpoint;
    }

    //求控制量
    pub fn next1(&mut self, value: T, setpoint: T) -> T {
        self.setpoint = setpoint;
        self.next(value)
    }

    //求控制量
    pub fn next(&mut self, value: T) -> T {
        //存放过去两次计算的误差值
        self.error.prev = self.error.last;
        self.error.last = self.error.current;
        //当前误差
        self.error.current = self.setpoint - value;
        match self.mode {
            //位置式PID
            Mode::Position => {
                //比例项输出
                self.p_out = limit(self.kp * self.error.current, self.limit_p.as_ref());
                //积分项限幅输出
                self.i_out = limit(
                    self.i_out + self.ki * self.error.current,
                    self.limit_i.as_ref(),
                );
                //存放过去两次计算的微分误差值
                self.derror.prev = self.derror.last;
                self.derror.last = self.derror.current;
                //当前误差的微分用本次误差减去上一次误差来计算
                self.derror.current = self.error.current - self.error.last;
                //微分项限幅输出
                self.d_out = limit(self.kd * self.derror.current, self.limit_d.as_ref());
                //叠加三个输出到总输出
                self.out = limit(
                    self.p_out + self.i_out + self.d_out,
                    self.limit_out.as_ref(),
                );
            }
            //增量式PID
            Mode::Increasing => {
                //以本次误差与上次误差的差值作为比例项的输入带入计算
                self.p_out = limit(
                    self.kp * (self.error.current - self.error.last),
                    self.limit_p.as_ref(),
                );
                //以本次误差作为积分项带入计算
                self.i_out = limit(self.ki * self.error.current, self.limit_i.as_ref());
                //迭代微分项的数组
                self.derror.prev = self.derror.last;
                self.derror.last = self.derror.current;
                //以本次误差与上次误差的差值减去上次误差与上上次误差的差值作为微分项的输入带入计算
                self.derror.current =
                    self.error.current - T::from(2.0).unwrap() * self.error.last + self.error.prev;
                self.d_out = limit(self.kd * self.derror.current, self.limit_d.as_ref());
                //叠加三个项的输出作为总输出
                self.out = self.out + self.p_out + self.i_out + self.d_out;
                //总输出限幅
                self.out = limit(self.out, self.limit_out.as_ref());
            }
        }
        self.out
    }

    pub fn reset(&mut self) {
        self.error.current = T::zero();
        self.error.last = T::zero();
        self.error.prev = T::zero();

        self.derror.current = T::zero();
        self.derror.last = T::zero();
        self.derror.prev = T::zero();
        self.out = T::zero();
    }
}

#[inline]
fn limit<T: FloatCore>(val: T, limit: Option<&Limit<T>>) -> T {
    if let Some(limit) = limit {
        if val > limit.max {
            limit.max
        } else if val < limit.min {
            limit.min
        } else {
            val
        }
    } else {
        val
    }
}
