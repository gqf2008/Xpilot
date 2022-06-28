/// 低通滤波
///
use super::Filter;

pub struct LowPassFilter {
    value: f32,
    a: f32,
}

impl LowPassFilter {
    /// value 已有值
    /// a 滤波系数0.0-1.0
    pub const fn new(value: f32, a: f32) -> Self {
        Self { value, a }
    }
}

impl Filter<f32, f32> for LowPassFilter {
    fn do_filter(&mut self, input: f32, output: &mut f32) {
        let val = self.a * input + (1.0 - self.a) * self.value;
        *output = val;
    }
}
