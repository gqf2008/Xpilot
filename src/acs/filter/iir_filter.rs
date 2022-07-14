//! 无限脉冲响应滤波器
//! 主要滤掉原始数据中的毛刺噪声数据。需要注意的是，它的输出参
//! 考了上次滤波数据，因此滤波阶数越高，滤掉毛刺噪声的能力越强，
//! 但对应的数据实时性会变差
//!
use super::Filter;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct IIRFilter {
    last_in: f32,  //上一次输入
    last_out: f32, //上一次输出
    n: f32,        //阶数
}

impl IIRFilter {
    pub const fn new(n: f32) -> Self {
        Self {
            last_in: 0.0,
            last_out: 0.0,
            n,
        }
    }
}

impl Filter<f32, f32> for IIRFilter {
    fn do_filter(&mut self, input: f32, output: &mut f32) {
        //TODO
        self.last_in = input;
        self.last_out = *output;
    }
}

pub struct IIRFilter3 {
    filters: [IIRFilter; 3],
}

impl IIRFilter3 {
    pub fn new(n: f32) -> Self {
        let filters = [IIRFilter::new(n); 3];
        Self { filters }
    }
}
impl Filter<Vector3<f32>, Vector3<f32>> for IIRFilter3 {
    fn do_filter(&mut self, input: Vector3<f32>, output: &mut Vector3<f32>) {
        self.filters
            .iter_mut()
            .enumerate()
            .for_each(|(i, f)| f.do_filter(input[i], &mut output[i]))
    }
}
