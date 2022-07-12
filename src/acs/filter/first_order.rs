/// ### 一阶滤波法
///
/// #### 优点
///
///  对周期性干扰具有良好的抑制作用；
/// 适用于波动频率较高的场合。
/// #### 缺点
///
///  相位滞后，灵敏度低；
/// 滞后程度取决于a值大小；
/// 不能消除滤波频率高于采样频率1/2的干扰信号
use super::Filter;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct FirstOrderFilter {
    a: f32,
    value: f32,
}

impl FirstOrderFilter {
    /// a 滤波系数，0.0-1.0
    /// 系数越小，滤波结果越平稳，但是灵敏度越低；
    /// 系数越大，灵敏度越高，但是滤波结果越不稳定。
    pub const fn new(a: f32) -> Self {
        Self { a, value: 0.0 }
    }
}

impl Filter<f32, f32> for FirstOrderFilter {
    fn do_filter(&mut self, input: f32, output: &mut f32) {
        self.value = input * self.a + (1.0 - self.a) * self.value;
        *output = self.value;
    }
}

pub struct FirstOrderFilter3 {
    filters: [FirstOrderFilter; 3],
}
impl FirstOrderFilter3 {
    pub fn new(a: f32) -> Self {
        Self {
            filters: [FirstOrderFilter::new(a); 3],
        }
    }
}
impl Filter<Vector3<f32>, Vector3<f32>> for FirstOrderFilter3 {
    fn do_filter(&mut self, input: Vector3<f32>, output: &mut Vector3<f32>) {
        self.filters
            .iter_mut()
            .enumerate()
            .for_each(|(i, f)| f.do_filter(input[i], &mut output[i]))
    }
}
