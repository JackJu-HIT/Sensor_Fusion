#include "sensorfusion.h"
/*****
 *HITWH606
 *Jack Ju
 *fusion filter 
 ******/ 
/****
 * Lidar只能够测量位置，字母L之后的数据依次为障碍物在X方向上的测量值（单位：米），
 * Y方向上的测量值（单位：米），测量时刻的时间戳（单位：微秒），
 * 障碍物位置在X方向上的真值（单位：米），障碍物位置在Y方向上的真值（单位：米），
 * 障碍物速度在X方向上的真值（单位：米/秒），障碍物速度在Y方向上的真值（单位：米/秒）。

 * Radar能够测量径向距离、角度和速度，
 * 字母R之后的数据依次为障碍物在极坐标系下的距离（单位：米），
 * 角度（单位：弧度），镜像速度（单位：米/秒），
 * 测量时刻的时间戳（单位：微秒），障碍物位置在X方向上的真值（单位：米），
 * 障碍物位置在Y方向上的真值（单位：米），障碍物速度在X方向上的真值（单位：米/秒），
 * 障碍物速度在Y方向上的真值（单位：米/秒）。
*****/

SensorFusion::SensorFusion()
{
    is_initialized_ = false;
    last_timestamp_ = 0.0;

    // 初始化激光雷达的测量矩阵 H_lidar_
    // Set Lidar's measurement matrix H_lidar_
    H_lidar_ = Eigen::MatrixXd(2, 4);
    H_lidar_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // 设置传感器的测量噪声矩阵，一般由传感器厂商提供，如不提供，也可通过有经验的工程师调试得到
    // Set R. R is provided by Sensor supplier, in sensor datasheet
    // set measurement covariance matrix
    R_lidar_ = Eigen::MatrixXd(2, 2);
    R_lidar_ << 0.0225, 0,
                0, 0.0225;

    // Measurement covariance matrix - radar
    R_radar_ = Eigen::MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
}

SensorFusion::~SensorFusion()
{

}

void SensorFusion::Process(MeasurementPackage measurement_pack)
{
    // 第一帧数据用于初始化 Kalman 滤波器
    if (!is_initialized_) {
        Eigen::Vector4d x;
        if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // 如果第一帧数据是激光雷达数据，没有速度信息，因此初始化时只能传入位置，速度设置为0
            x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // 如果第一帧数据是毫米波雷达，可以通过三角函数算出x-y坐标系下的位置和速度
            float rho = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];
            float rho_dot = measurement_pack.raw_measurements_[2];
            float position_x = rho * cos(phi);
            if (position_x < 0.0001) {
                position_x = 0.0001;
            }
            float position_y = rho * sin(phi);
            if (position_y < 0.0001) {
                position_y = 0.0001;
            }
            float velocity_x = rho_dot * cos(phi);
            float velocity_y = rho_dot * sin(phi);
            x << position_x, position_y, velocity_x , velocity_y;
        }
        
        // 避免运算时，0作为被除数
        if (fabs(x(0)) < 0.001) {
            x(0) = 0.001;
        }
        if (fabs(x(1)) < 0.001) {
            x(1) = 0.001;
        }
        // 初始化Kalman滤波器
        kf_.Initialization(x);

        // 设置协方差矩阵P
        Eigen::MatrixXd P = Eigen::MatrixXd(4, 4);
        P << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1000.0, 0.0,
             0.0, 0.0, 0.0, 1000.0;
        kf_.SetP(P);

        // 设置过程噪声Q
        Eigen::MatrixXd Q = Eigen::MatrixXd(4, 4);
        Q << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
        kf_.SetQ(Q);

        // 存储第一帧的时间戳，供下一帧数据使用
        last_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;//设置成True以后，后续的就执行非第一帧的情况了。
        return;
    }

    /****** is_initialized_ = true;//设置成True以后，后续的就执行如下的情况*****/


    // 求前后两帧的时间差，数据包中的时间戳单位为微秒，处以1e6，转换为秒
    double delta_t = (measurement_pack.timestamp_ - last_timestamp_) / 1000000.0; // unit : s
    last_timestamp_ = measurement_pack.timestamp_;

    // 设置状态转移矩阵F
    Eigen::MatrixXd F = Eigen::MatrixXd(4, 4);
    F << 1.0, 0.0, delta_t, 0.0,
         0.0, 1.0, 0.0, delta_t,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    kf_.SetF(F);

    // 预测
    kf_.Prediction();

    // 更新
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        kf_.SetH(H_lidar_);     //代表测量矩阵
        kf_.SetR(R_lidar_);     //代表测量的方差矩阵
        kf_.KFUpdate(measurement_pack.raw_measurements_);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        kf_.SetR(R_radar_);
        // Jocobian矩阵Hj的运算已包含在EKFUpdate中
        kf_.EKFUpdate(measurement_pack.raw_measurements_);
    }
}
