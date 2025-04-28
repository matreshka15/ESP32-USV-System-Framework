#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

typedef struct {
    float q; // 过程噪声协方差
    float r; // 测量噪声协方差
    float x; // 估计值
    float p; // 估计误差协方差
    float k; // 卡尔曼增益
} KalmanFilter;

void kalman_filter_init(KalmanFilter *kf, float q, float r, float initial_x, float initial_p);
float kalman_filter_update(KalmanFilter *kf, float measurement);

#endif