#include "kalman_filter.h"

void kalman_filter_init(KalmanFilter *kf, float q, float r, float initial_x, float initial_p) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_x;
    kf->p = initial_p;
}

float kalman_filter_update(KalmanFilter *kf, float measurement) {
    // 预测步骤
    kf->p = kf->p + kf->q;

    // 更新步骤
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1 - kf->k) * kf->p;

    return kf->x;
}