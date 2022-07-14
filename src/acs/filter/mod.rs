//! 滤波抽象&常用实现
//! 参考链接: https://zhuanlan.zhihu.com/p/271154535

pub mod ahrs;
pub mod dither;
pub mod first_order;
pub mod iir_filter;
pub mod jitter_filter;
pub mod limiting;
pub mod limiting_dither;
pub mod limiting_mean_values;
pub mod low_pass;
pub mod mean;
pub mod mean_mean;
pub mod mean_value;
pub mod moving_average;
pub mod weighted_moving_average;

pub trait Filter<In, Out>: Send + Sync {
    fn do_filter(&mut self, input: In, output: &mut Out);

    fn chain<R: Filter<In, Out>>(self, next: R) -> Chain<Self, R>
    where
        Self: Sized,
    {
        Chain {
            first: self,
            second: next,
        }
    }
}

#[derive(Default, Clone, Copy)]
pub struct Chain<T, U> {
    first: T,
    second: U,
}

impl<T, U> Chain<T, U> {
    pub fn into_inner(self) -> (T, U) {
        (self.first, self.second)
    }

    pub fn get_ref(&self) -> (&T, &U) {
        (&self.first, &self.second)
    }
    pub fn get_mut(&mut self) -> (&mut T, &mut U) {
        (&mut self.first, &mut self.second)
    }
}

impl<In: Copy, Out, T: Filter<In, Out>, U: Filter<In, Out>> Filter<In, Out> for Chain<T, U> {
    fn do_filter(&mut self, input: In, output: &mut Out) {
        self.first.do_filter(input, output);
        self.second.do_filter(input, output);
    }
}
