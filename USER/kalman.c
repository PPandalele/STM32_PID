#include "kalman.h"


// ��ʼ���������˲���,ѹ��
void p_kalman_filter_init(KalmanFilter *kf, double initial_estimate, double variance, double gain,double error_estimate) {  
    kf->last_estimate = initial_estimate;        // ��һ�εĹ���ֵ 
    kf->estimate = initial_estimate;             // ��ǰ�Ĺ���ֵ  
    kf->variance = variance;                     // ���Ƶ��������� 
    kf->kalman_gain = gain;                      // ����������  
    kf->error_estimate = error_estimate;         // ���Ĺ���ֵ  
} 

// ��ʼ���������˲���,����
void f_kalman_filter_init(KalmanFilter *kf, double initial_estimate, double variance, double gain,double error_estimate) {  
    kf->last_estimate = initial_estimate;        // ��һ�εĹ���ֵ 
    kf->estimate = initial_estimate;             // ��ǰ�Ĺ���ֵ  
    kf->variance = variance;                     // ���Ƶ��������� 
    kf->kalman_gain = gain;                      // ����������  
    kf->error_estimate = error_estimate;         // ���Ĺ���ֵ  
} 


// �������˲����ĸ��º���  
double kalman_filter_update(KalmanFilter *kf, double measurement) {  
    // ���㿨��������  
    kf->kalman_gain = kf->error_estimate / (kf->error_estimate + kf->variance);  
  
    // �������Ĺ���ֵ  
    kf->error_estimate = (1 - kf->kalman_gain) * kf->error_estimate + kf->kalman_gain * (measurement - kf->last_estimate);  
  
    // ���¹���ֵ  
    kf->estimate = kf->last_estimate + kf->kalman_gain * (measurement - kf->last_estimate);  
  
    // �������һ�εĹ���ֵ  
    kf->last_estimate = kf->estimate;  
  
    return kf->estimate;  
}


//int main() {  
//    // ��ʼ���������˲����������ʼ�¶�Ϊ0����������Ϊ0.1  
//    KalmanFilter kf;  
//    kalman_filter_init(&kf, 0.0, 0.1);  
//  
//    // ģ��һϵ�е��¶Ȳ���ֵ  
//    double measurements[] = {0.1, 0.2, 0.3, 0.4, 0.5};  
//    int n = sizeof(measurements) / sizeof(measurements[0]);  
//  
//    // ��ÿ������ֵ���п������˲�  
//    for (int i = 0; i < n; i++) {  
//        double filtered_value = kalman_filter_update(&kf, measurements[i]);  
//        printf("Measurement: %lf, Filtered value: %lf\n", measurements[i], filtered_value);  
//    }  
//  
//    return 0;  
//}










