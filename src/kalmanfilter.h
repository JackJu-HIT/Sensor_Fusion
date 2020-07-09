/*****
 *HITWH606
 *Jack Ju
 *kalman filter header
 ******/ 
#pragma once
#include "Eigen/Dense"

class KalmanFilter {
public:

    KalmanFilter();
    ~KalmanFilter();
    //Vector3d 本质上还是Eigen::Matrix<double,3,1>即三维向量
    void Initialization(Eigen::VectorXd x_in);

    bool IsInitialized();

    void SetF(Eigen::MatrixXd F_in);

    void SetP(Eigen::MatrixXd P_in);

    void SetQ(Eigen::MatrixXd Q_in);

    void SetH(Eigen::MatrixXd H_in);

    void SetR(Eigen::MatrixXd R_in);

    void Prediction();

    void KFUpdate(Eigen::VectorXd z);

    void EKFUpdate(Eigen::VectorXd z);

    Eigen::VectorXd GetX();

private:

    void CalculateJacobianMatrix();

    // flag of initialization
    bool is_initialized_;

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transistion matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;
};
