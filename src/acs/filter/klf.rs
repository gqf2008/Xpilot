use super::Filter;
use alloc::vec;

use nalgebra::{DMatrix, Dynamic, VecStorage};
// 卡尔曼滤波的本质是通过k系数来表示更相信哪个值；预估值=值1 + k * (值1 - 值2);
// 所以核心就是怎么计算k值

//简单卡尔曼算法
//用前一次的值和当前值估计最优值
#[derive(Debug, Clone, Copy)]
pub struct KalmanFilter {
    k: f32,                   //k时刻的卡尔曼增益K，0-1之间
    last_error_estimate: f32, //上一次的估计误差
    error_measurement: f32,   //测量误差，可以通过实验测出来
    last: f32,                //上一次的估计值
}

impl KalmanFilter {
    //error_measurement 测量误差，可以通过实验测出来
    pub const fn new(error_measurement: f32) -> Self {
        Self {
            k: 0.0,
            last: 0.0,
            last_error_estimate: 1.0,
            error_measurement,
        }
    }
}

impl Filter<f32, f32> for KalmanFilter {
    // z测量值
    fn do_filter(&mut self, z: f32, x: &mut f32) {
        //第一步: 计算k时刻卡尔曼增益=上一次的估计误差/（上一次的估计误差+测量误差）
        self.k = self.last_error_estimate / (self.last_error_estimate + self.error_measurement);
        //第二步: 计算估计值，当前的估计值=上一次的估计值+k*(当前测量值-上一次的估计值)
        *x = self.last + self.k * (z - self.last);
        //第三步: 更新估计误差
        self.last_error_estimate = (1.0 - self.k) * self.last_error_estimate;
        self.last = *x;
    }
}

//卡尔曼融合
//两个传感器的采样数据估计最优值
#[derive(Debug, Clone, Copy)]
pub struct FusionKalmanFilter {
    k: f32,  //k时刻的卡尔曼增益K，0-1之间
    sd: f32, //最优估计的标准差
}

impl FusionKalmanFilter {
    pub fn new() -> Self {
        Self { k: 0.0, sd: 0.0 }
    }

    //使用样本校准，先计算方差再计算k
    pub fn calibrate(&mut self, sample: &[(f32, f32)]) {
        //第一步: 求平均数
        let (sum1, sum2) = sample.iter().fold((0.0, 0.0), |(sum1, sum2), (v1, v2)| {
            (sum1 + *v1, sum2 + *v2)
        });
        let (avg1, avg2) = (sum1 / sample.len() as f32, sum2 / sample.len() as f32);
        //第二步: 求方差
        let (var1_sum, var2_sum) =
            sample
                .iter()
                .fold((0.0, 0.0), |(var1_sum, var2_sum), (v1, v2)| {
                    (
                        var1_sum + (*v1 - avg1) * (*v1 - avg1),
                        var2_sum + (*v2 - avg2) * (*v2 - avg2),
                    )
                });
        let (var1, var2) = (
            var1_sum / sample.len() as f32,
            var2_sum / sample.len() as f32,
        );
        //第三步: 求卡尔曼增益K
        //k=标准差1的平方/(标准差1的平方+标准差2的平方); 方差=标准差的平方
        let k = var1 / (var1 + var2);
        //第四步: 求标准差，先求方差，方差开根号就是标准差
        //方差=(1-k)的平方*方差1+k的平方*方差2
        let sd = libm::sqrtf((1.0 - k) * (1.0 - k) * var1 + k * k * var2);
        self.k = k;
        self.sd = sd;
    }
    //返回最优估计的标准差
    pub fn standard_deviation(&self) -> f32 {
        self.sd
    }
}

impl Filter<(f32, f32), f32> for FusionKalmanFilter {
    // 根据两个输入量估计最优值
    fn do_filter(&mut self, z: (f32, f32), x: &mut f32) {
        *x = z.0 + self.k * (z.1 - z.0);
    }
}

//卡尔曼滤波
#[derive(Debug, Clone)]
pub struct MatrixKalmanFilter {
    // standard_deviation: f32, //标准差
    q: DMatrix<f32>, //过程误差/过程激励噪声协方差矩阵
    r: DMatrix<f32>, //测量误差/观测噪声协方差矩阵
    a: DMatrix<f32>, //状态转移矩阵
    h: DMatrix<f32>, //观测器矩阵
    i: DMatrix<f32>, //单位矩阵

    k: DMatrix<f32>, //k时刻卡尔曼增益K，0-1之间，k=标准方差1的平方/(标准方差1的平方+标准方差2的平方)
    p_: DMatrix<f32>, //先验估计误差协方差矩阵
    p: DMatrix<f32>, //后验估计误差协方差矩阵
    x_: DMatrix<f32>, //先验估计
    x: DMatrix<f32>, //后验估计
    qe_sd: DMatrix<f32>, //过程误差标准方差
    re_sd: DMatrix<f32>, //测量误差标准方差
    qe_w: DMatrix<f32>, //状态过程误差
    re_v: DMatrix<f32>, //测量过程误差
}

impl MatrixKalmanFilter {
    pub fn new(
        q: DMatrix<f32>,
        r: DMatrix<f32>,
        a: DMatrix<f32>,
        h: DMatrix<f32>,
        i: DMatrix<f32>,
    ) -> Self {
        Self {
            q,
            r,
            a,
            h,
            i,
            k: DMatrix::from_vec_storage(VecStorage::new(Dynamic::new(0), Dynamic::new(0), vec![])),
            p_: DMatrix::from_vec_storage(VecStorage::new(
                Dynamic::new(0),
                Dynamic::new(0),
                vec![],
            )),
            p: DMatrix::from_vec_storage(VecStorage::new(Dynamic::new(0), Dynamic::new(0), vec![])),
            x_: DMatrix::from_vec_storage(VecStorage::new(
                Dynamic::new(0),
                Dynamic::new(0),
                vec![],
            )),
            x: DMatrix::from_vec_storage(VecStorage::new(Dynamic::new(0), Dynamic::new(0), vec![])),
            qe_sd: DMatrix::from_vec_storage(VecStorage::new(
                Dynamic::new(0),
                Dynamic::new(0),
                vec![],
            )),
            re_sd: DMatrix::from_vec_storage(VecStorage::new(
                Dynamic::new(0),
                Dynamic::new(0),
                vec![],
            )),
            qe_w: DMatrix::from_vec_storage(VecStorage::new(
                Dynamic::new(0),
                Dynamic::new(0),
                vec![],
            )),
            re_v: DMatrix::from_vec_storage(VecStorage::new(
                Dynamic::new(0),
                Dynamic::new(0),
                vec![],
            )),
        }
    }
}

impl Filter<DMatrix<f32>, DMatrix<f32>> for MatrixKalmanFilter {
    fn do_filter(&mut self, z: DMatrix<f32>, x: &mut DMatrix<f32>) {
        let z = z.clone() + z;
        let z = self.k.clone() * z.clone() + z;
        *x = z;
    }
}
