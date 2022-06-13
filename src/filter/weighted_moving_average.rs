/// ### 加权递推平均滤波法（加权滑动平均滤波）
///
/// 是对递推平均滤波法的改进，即不同时刻的数据加以不同的权；
/// 通常是，越接近现时刻的数据，权取得越大。
/// 给予新采样值的权系数越大，则灵敏度越高，但信号平滑度越低。
/// #### 优点
///
///  适用于有较大纯滞后时间常数的对象，和采样周期较短的系统。
/// #### 缺点
///
///  对于纯滞后时间常数较小、采样周期较长、变化缓慢的信号；
/// 不能迅速反应系统当前所受干扰的严重程度，滤波效果差
use super::Filter;

pub struct WeightedMovingAverageFilter<const N: usize> {
    values: [f32; N],
    sum_coe: u32,
}
impl<const N: usize> WeightedMovingAverageFilter<N> {
    pub const fn new(cos: [f32; N], sum_coe: u32) -> Self {
        Self {
            values: cos,
            sum_coe,
        }
    }
}

impl<const N: usize> Filter<f32, f32> for WeightedMovingAverageFilter<N> {
    fn do_filter(&mut self, input: f32, output: &mut f32) {
        self.values[N - 1] = input;
        let mut sum = 0.0;
        for i in 0..self.values.len() - 1 {
            let val = self.values[i + 1];
            sum += val * (i + 1) as f32;
            self.values[i] = val;
        }
        *output = sum / self.sum_coe as f32;
    }
}
