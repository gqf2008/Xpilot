/// ### 限幅消抖滤波法
///
/// 相当于“限幅滤波法”+“消抖滤波法”；
/// 先限幅，后消抖。
/// #### 优点
///
/// 继承了“限幅”和“消抖”的优点；
/// 改进了“消抖滤波法”中的某些缺陷，避免将干扰值导入系统。
/// #### 缺点
///
/// 对于快速变化的参数不宜。
///
use super::{dither::DitherFilter, limiting::LimitingFilter, Chain, Filter};
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct LimitingDitherFilter<const N: usize>;

impl<const N: usize> LimitingDitherFilter<N> {
    pub fn new(limit: f32) -> Chain<LimitingFilter, DitherFilter<N>> {
        LimitingFilter::new(limit).chain(DitherFilter::new())
    }
}

pub struct LimitingDitherFilter3<const N: usize> {
    filters: [Chain<LimitingFilter, DitherFilter<N>>; 3],
}

impl<const N: usize> LimitingDitherFilter3<N> {
    pub fn new(limit: f32) -> Self {
        let filters = [LimitingDitherFilter::<N>::new(limit); 3];
        Self { filters }
    }
}

impl<const N: usize> Filter<Vector3<f32>, Vector3<f32>> for LimitingDitherFilter3<N> {
    fn do_filter(&mut self, input: Vector3<f32>, output: &mut Vector3<f32>) {
        self.filters
            .iter_mut()
            .enumerate()
            .for_each(|(i, f)| f.do_filter(input[i], &mut output[i]))
    }
}
