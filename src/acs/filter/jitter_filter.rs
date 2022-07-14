//!抖动滤波
//! 在原始数据处于微小抖动了，为了实现滤波数据跟随，同时
//! 又不受微小抖动而波动，通常会实现抖动滤波。抖动滤波简
//! 单的说，就是当前依次原始数据，与上次滤波后的数据的差
//! 值，超过某一限定，才调整滤波输出数据，从而滤波原始数
//! 据中存在的微小抖动信号。抖动滤波一般放在其他滤波方法
//! 之后使用，进一步稳定滤波后的数据。
//!
use super::Filter;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct JitterFilter {
    threshold: f32,
    last: f32,
}

impl JitterFilter {
    pub const fn new(threshold: f32) -> Self {
        Self {
            last: 0.0,
            threshold,
        }
    }
}

impl Filter<f32, f32> for JitterFilter {
    fn do_filter(&mut self, input: f32, output: &mut f32) {
        let error = input - self.last;
        if error >= self.threshold {
            *output = input - self.threshold;
        } else if error <= -self.threshold {
            *output = input + self.threshold;
        } else {
            *output = self.last;
        }
        self.last = *output;
    }
}

pub struct JitterFilter3 {
    filters: [JitterFilter; 3],
}

impl JitterFilter3 {
    pub fn new(threshold: f32) -> Self {
        let filters = [JitterFilter::new(threshold); 3];
        Self { filters }
    }
}
impl Filter<Vector3<f32>, Vector3<f32>> for JitterFilter3 {
    fn do_filter(&mut self, input: Vector3<f32>, output: &mut Vector3<f32>) {
        self.filters
            .iter_mut()
            .enumerate()
            .for_each(|(i, f)| f.do_filter(input[i], &mut output[i]))
    }
}
