#include "kalman.h"


// 初始化卡尔曼滤波器,压力
void p_kalman_filter_init(KalmanFilter *kf, double initial_estimate, double variance, double gain,double error_estimate) {  
    kf->last_estimate = initial_estimate;        // 上一次的估计值 
    kf->estimate = initial_estimate;             // 当前的估计值  
    kf->variance = variance;                     // 估计的噪声方差 
    kf->kalman_gain = gain;                      // 卡尔曼增益  
    kf->error_estimate = error_estimate;         // 误差的估计值  
} 

// 初始化卡尔曼滤波器,流量
void f_kalman_filter_init(KalmanFilter *kf, double initial_estimate, double variance, double gain,double error_estimate) {  
    kf->last_estimate = initial_estimate;        // 上一次的估计值 
    kf->estimate = initial_estimate;             // 当前的估计值  
    kf->variance = variance;                     // 估计的噪声方差 
    kf->kalman_gain = gain;                      // 卡尔曼增益  
    kf->error_estimate = error_estimate;         // 误差的估计值  
} 


// 卡尔曼滤波器的更新函数  
double kalman_filter_update(KalmanFilter *kf, double measurement) {  
    // 计算卡尔曼增益  
    kf->kalman_gain = kf->error_estimate / (kf->error_estimate + kf->variance);  
  
    // 更新误差的估计值  
    kf->error_estimate = (1 - kf->kalman_gain) * kf->error_estimate + kf->kalman_gain * (measurement - kf->last_estimate);  
  
    // 更新估计值  
    kf->estimate = kf->last_estimate + kf->kalman_gain * (measurement - kf->last_estimate);  
  
    // 更新最后一次的估计值  
    kf->last_estimate = kf->estimate;  
  
    return kf->estimate;  
}


//int main() {  
//    // 初始化卡尔曼滤波器，假设初始温度为0，噪声方差为0.1  
//    KalmanFilter kf;  
//    kalman_filter_init(&kf, 0.0, 0.1);  
//  
//    // 模拟一系列的温度测量值  
//    double measurements[] = {0.1, 0.2, 0.3, 0.4, 0.5};  
//    int n = sizeof(measurements) / sizeof(measurements[0]);  
//  
//    // 对每个测量值进行卡尔曼滤波  
//    for (int i = 0; i < n; i++) {  
//        double filtered_value = kalman_filter_update(&kf, measurements[i]);  
//        printf("Measurement: %lf, Filtered value: %lf\n", measurements[i], filtered_value);  
//    }  
//  
//    return 0;  
//}










