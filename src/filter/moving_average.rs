/// ### 递推平均滤波法（滑动平均滤波）
///
/// 把连续N个采样值看成一个队列，队列长度固定为N；
/// 每次采样到一个新数据放入队尾，并扔掉队首的一次数据。把队列中的N各数据进行平均运算，即获得新的滤波结果
/// #### 优点
///
/// 对周期性干扰有良好的抑制作用，平滑度高； 适用于高频振荡的系统。
/// ####缺点
///
/// 灵敏度低；
/// 对偶然出现的脉冲性干扰的抑制作用较差，不适于脉冲干扰较严重的场合 不适合用于开关电源电路
use super::Filter;

pub struct MovingAverageFilter<const N: usize> {
    values: [f32; N],
}
impl<const N: usize> MovingAverageFilter<N> {
    pub fn new() -> Self {
        Self { values: [0.0; N] }
    }
}

impl<const N: usize> Filter<f32, f32> for MovingAverageFilter<N> {
    fn do_filter(&mut self, input: f32, output: &mut f32) {
        self.values[N - 1] = input;
        let mut sum = 0.0;
        for i in 0..self.values.len() - 1 {
            let val = self.values[i + 1];
            sum += val;
            self.values[i] = val;
        }

        *output = sum / (N - 1) as f32;
    }
}
