/// ### 中位值平均滤波法
///
/// 采一组队列去掉最大值和最小值
/// #### 优点
///
/// 融合了两种滤波的优点。对于偶然出现的脉冲性干扰，可消除有其引起的采样值偏差。
/// 对周期干扰有良好的抑制作用，平滑度高，适于高频振荡的系统。
/// #### 缺点
///
/// 测量速度慢。
use super::Filter;

pub struct MeanMeanFilter;

impl Filter<&mut [f32], f32> for MeanMeanFilter {
    fn do_filter(&mut self, input: &mut [f32], output: &mut f32) {
        let mut temp: f32;
        let n = input.len();
        // TODO 用迭代器避免数组越界检查开销
        for j in 0..n - 1 {
            for i in 0..n - j {
                if input[i] > input[i + 1] {
                    temp = input[i];
                    input[i] = input[i + 1];
                    input[i + 1] = temp;
                }
            }
        }
        let mut sum = 0.0;
        for i in 0..n - 1 {
            sum += input[i];
        }
        *output = sum / (n - 2) as f32
    }
}
