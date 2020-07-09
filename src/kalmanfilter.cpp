/*****
 *HITWH606
 *Jack Ju
 *kalman filter 
 ******/ 
/***
 * 程序说明：
 * 本程序是对kalmanfilter.h中的函数具体的实现
 * **/
#include "kalmanfilter.h"
#include "iostream"
using namespace std;
KalmanFilter::KalmanFilter()
{
    is_initialized_ = false;
}

KalmanFilter::~KalmanFilter()
{

}

void KalmanFilter::Initialization(Eigen::VectorXd x_in)
{
    x_ = x_in;
}

bool KalmanFilter::IsInitialized()
{
    return is_initialized_;
}

void KalmanFilter::SetF(Eigen::MatrixXd F_in)
{
    F_ = F_in;
}

void KalmanFilter::SetP(Eigen::MatrixXd P_in)
{
    P_ = P_in;
}

void KalmanFilter::SetQ(Eigen::MatrixXd Q_in)
{
    Q_ = Q_in;
}

void KalmanFilter::SetH(Eigen::MatrixXd H_in)
{
    H_ = H_in;
}

void KalmanFilter::SetR(Eigen::MatrixXd R_in)
{
    R_ = R_in;
}

void KalmanFilter::Prediction()
{
    x_ = F_ * x_;//模型可修改
    Eigen::MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
    //cout<<P_<<endl;
}

void KalmanFilter::KFUpdate(Eigen::VectorXd z)
{
    Eigen::VectorXd y = z - H_ * x_;
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd K =  P_ * Ht * Si;
    x_ = x_ + (K * y);
    int x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);//单位阵
    P_ = (I - K * H_) * P_; 
}

void KalmanFilter::EKFUpdate(Eigen::VectorXd z)
{
    double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    double theta = atan2(x_(1), x_(0));   //atan(y,x)
    double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    Eigen::VectorXd h = Eigen::VectorXd(3);
    h << rho, theta, rho_dot;
    Eigen::VectorXd y = z - h;

    CalculateJacobianMatrix();

    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd K =  P_ * Ht * Si;
    x_ = x_ + (K * y);
    int x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::GetX()
{
    return x_;
}

void KalmanFilter::CalculateJacobianMatrix()
{
    Eigen::MatrixXd Hj(3, 4);

    // get state parameters
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    // pre-compute a set of terms to avoid repeated calculation
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = (c1 * c2);

    // Check division by zero
    if(fabs(c1) < 0.0001){
        H_ = Hj;
        return;
    }

    Hj << (px/c2), (py/c2), 0, 0,
         -(py/c1), (px/c1), 0, 0,
          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    H_ = Hj;
}
