#ifndef __KALMAN_H__
#define __KALMAN_H__

// �������˲����Ľṹ  
typedef struct {  
    double last_estimate;     // ��һ�εĹ���ֵ  
    double estimate;          // ��ǰ�Ĺ���ֵ  
    double variance;          // ���Ƶ���������  
    double kalman_gain;       // ����������  
    double error_estimate;    // ���Ĺ���ֵ  
} KalmanFilter;


double kalman_filter_update(KalmanFilter *kf, double measurement);
void p_kalman_filter_init(KalmanFilter *kf, double initial_estimate, double variance, double gain,double error_estimate);
void f_kalman_filter_init(KalmanFilter *kf, double initial_estimate, double variance, double gain,double error_estimate);


#endif

