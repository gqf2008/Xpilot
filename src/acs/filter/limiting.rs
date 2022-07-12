/// ### 限幅滤波
///
/// #### 优点
///
/// 能有效克服因偶然因素引起的脉冲干扰。
///
/// #### 缺点
///
/// 无法抑制那种周期性的干扰，且平滑度差
use super::Filter;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct LimitingFilter {
    value: f32,
    limit: f32,
}

impl LimitingFilter {
    pub const fn new(limit: f32) -> Self {
        Self { value: 0.0, limit }
    }
}

impl Filter<f32, f32> for LimitingFilter {
    fn do_filter(&mut self, input: f32, output: &mut f32) {
        if self.value == 0.0 {
            self.value = input;
        }
        if (input - self.value >= self.limit) || (self.value - input >= self.limit) {
            *output = self.value;
        } else {
            *output = input;
            self.value = input;
        }
    }
}

pub struct LimitingFilter3 {
    filters: [LimitingFilter; 3],
}

impl LimitingFilter3 {
    pub fn new(limit: f32) -> Self {
        let filters = [LimitingFilter::new(limit); 3];
        Self { filters }
    }
}
impl Filter<Vector3<f32>, Vector3<f32>> for LimitingFilter3 {
    fn do_filter(&mut self, input: Vector3<f32>, output: &mut Vector3<f32>) {
        self.filters
            .iter_mut()
            .enumerate()
            .for_each(|(i, f)| f.do_filter(input[i], &mut output[i]))
    }
}
