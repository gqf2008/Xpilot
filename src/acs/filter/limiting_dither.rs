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

pub struct LimitingDitherFilter<const N: usize>;

impl<const N: usize> LimitingDitherFilter<N> {
    pub fn new(limit: f32) -> Chain<LimitingFilter, DitherFilter<N>> {
        LimitingFilter::new(limit).chain(DitherFilter::new())
    }
}
