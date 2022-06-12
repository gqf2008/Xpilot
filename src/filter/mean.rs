/// ### 中位值滤波法
///
/// #### 优点
///
/// 能有效克服因偶然因素引起的波动干扰；
/// 对温度、液位等变化缓慢的被测参数有良好的滤波效果。
/// #### 缺点
///
/// 对流量，速度等快速变化的参数不宜
use super::Filter;
pub struct MeanFilter;

impl Filter<&mut [f32], f32> for MeanFilter {
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
        *output = input[(n - 1) / 2];
    }
}
