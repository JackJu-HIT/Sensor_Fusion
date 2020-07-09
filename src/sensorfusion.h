#pragma once

#include "interface/measurement_package.h"
#include "kalmanfilter.h"

class SensorFusion {
public:
    SensorFusion();
    ~SensorFusion();

    void Process(MeasurementPackage measurement_pack);
    KalmanFilter kf_;//实例化创建一个卡尔曼滤波的对象

private:
    bool is_initialized_;
    long last_timestamp_;
    Eigen::MatrixXd R_lidar_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_lidar_;
};
