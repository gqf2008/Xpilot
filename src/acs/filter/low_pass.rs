/// 低通滤波
///
use super::Filter;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
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

pub struct LowPassFilter3 {
    filters: [LowPassFilter; 3],
}
impl LowPassFilter3 {
    pub fn new(value: f32, a: f32) -> Self {
        Self {
            filters: [LowPassFilter::new(value, a); 3],
        }
    }
}
impl Filter<Vector3<f32>, Vector3<f32>> for LowPassFilter3 {
    fn do_filter(&mut self, input: Vector3<f32>, output: &mut Vector3<f32>) {
        self.filters
            .iter_mut()
            .enumerate()
            .for_each(|(i, f)| f.do_filter(input[i], &mut output[i]))
    }
}
