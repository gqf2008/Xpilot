use alloc::vec;

/// ### 限幅平均滤波法
///
/// #### 优点
///
///  融合了两种滤波法的优点；
/// 对于偶然出现的脉冲性干扰，可消除有其引起的采样值偏差。
/// #### 缺点
///
///  比较浪费RAM。
///
use super::{limiting::LimitingFilter, mean_value::MeanValueFilter, Filter};

pub struct LimitingMeanValuesFilter {
    limit: LimitingFilter,
    mv: MeanValueFilter,
}

impl LimitingMeanValuesFilter {
    pub fn new(limit: f32) -> Self {
        Self {
            limit: LimitingFilter::new(limit),
            mv: MeanValueFilter,
        }
    }
}

impl Filter<&mut [f32], f32> for LimitingMeanValuesFilter {
    fn do_filter(&mut self, input: &mut [f32], output: &mut f32) {
        let mut values = vec![0.0; input.len()];
        for (i, v) in input.iter().enumerate() {
            self.limit.do_filter(*v, &mut values[i]);
        }
        self.mv.do_filter(&mut values, output)
    }
}
