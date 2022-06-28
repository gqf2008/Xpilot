/// ### 算数平均滤波法
///
/// #### 说明
///
/// 连续取N个采样值进行算术平均运算。
/// #### 优点
///
/// 试用于对一般具有随机干扰的信号进行滤波。
/// 这种信号的特点是有一个平均值，信号在某一数值范围附近上下波动。
/// #### 缺点
///
/// 对于测量速度较慢或要求数据计算较快的实时控制不适用
use super::Filter;

pub struct MeanValueFilter;

impl Filter<&mut [f32], f32> for MeanValueFilter {
    fn do_filter(&mut self, input: &mut [f32], output: &mut f32) {
        let sum = input.iter().fold(0.0, |acc, v| acc + v);
        *output = sum / input.len() as f32;
    }
}
