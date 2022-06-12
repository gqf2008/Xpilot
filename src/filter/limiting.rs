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

pub struct LimitingFilter {
    value: f32,
    limit: f32,
}

impl LimitingFilter {
    pub fn new(limit: f32) -> Self {
        Self { value: 0.0, limit }
    }
}

impl Filter<f32, f32> for LimitingFilter {
    fn do_filter(&mut self, input: f32, output: &mut f32) {
        if self.value == 0.0 {
            self.value = input;
        }
        if (input - self.value > self.limit) || (self.value - input > self.limit) {
            *output = self.value;
        } else {
            *output = input;
        }
    }
}
