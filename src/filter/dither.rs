/// ### 消抖滤波法
///
/// 设置一个滤波计数器，将每次采样值与当前有效值比较：
/// 如果采样值=当前有效值，则计数器清零；
/// 如果采样值<>当前有效值，则计数器+1，并判断计数器是否>=上限N（溢出）；
/// 如果计数器溢出，则将本次值替换当前有效值，并清计数器。
/// ### 优点
///
/// 对于变化缓慢的被测参数有较好的滤波效果；
/// 可避免在临界值附近控制器的反复开/关跳动或显示器上数值抖动
/// ### 缺点
///
///  对于快速变化的参数不宜；
/// 如果在计数器溢出的那一次采样到的值恰好是干扰值,则会将干扰值当作有效值导入系统。
///
use super::Filter;

pub struct DitherFilter<const N: usize> {
    value: f32,
    count: usize,
}

impl<const N: usize> DitherFilter<N> {
    pub fn new() -> Self {
        Self {
            value: 0.0,
            count: 0,
        }
    }
}

impl<const N: usize> Filter<f32, f32> for DitherFilter<N> {
    fn do_filter(&mut self, input: f32, output: &mut f32) {
        if self.value != input {
            self.count += 1;
            if self.count > N {
                self.count = 0;
                self.value = input;
            }
        } else {
            self.count = 0;
        }
        *output = self.value;
    }
}
