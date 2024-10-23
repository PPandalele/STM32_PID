#ifndef __KALMAN_H__
#define __KALMAN_H__

// 卡尔曼滤波器的结构  
typedef struct {  
    double last_estimate;     // 上一次的估计值  
    double estimate;          // 当前的估计值  
    double variance;          // 估计的噪声方差  
    double kalman_gain;       // 卡尔曼增益  
    double error_estimate;    // 误差的估计值  
} KalmanFilter;


double kalman_filter_update(KalmanFilter *kf, double measurement);
void p_kalman_filter_init(KalmanFilter *kf, double initial_estimate, double variance, double gain,double error_estimate);
void f_kalman_filter_init(KalmanFilter *kf, double initial_estimate, double variance, double gain,double error_estimate);


#endif

