/*****
 *HITWH606
 *Jack Ju
 *main procedure 
 ******/ 
#include <ros/ros.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include "Eigen/Eigen"
#include "interface/ground_truth_package.h"
#include "interface/measurement_package.h"
#include "sensorfusion.h"
#include "fusion_data.h"
#include "source_data.h"
using namespace std;
int main(int argc, char **argv) {

// ROS节点初始化
    ros::init(argc, argv, "fusion_data_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/odom_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher sensor_info_pub = n.advertise<sensor_fusion::fusion_data>("/fusion_data", 100);
     ros::Publisher source_info_pub = n.advertise<sensor_fusion::source_data>("/orgin_data", 100);
    // 设置循环的频率
    ros::Rate loop_rate(1);
    //ROS_INFO("The data of sensor_data is publishing!");此行必须注释掉，否则程序输出数据为无穷大。

      
//{//循环读取发送模拟数据


    // 设置毫米波雷达/激光雷达输入数据的路径
    // Set radar & lidar data file path
    std::string input_file_name = "/home/juchunyu/catkin_ws/src/sensor_fusion/src/data/sample-laser-radar-measurement-data-2.txt";
//"../data/sample-laser-radar-measurement-data-2.txt";

    // 打开数据，若失败则输出失败信息，返回-1，并终止程序
    // Open file. if failed return -1 & end program
    std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);
    if (!input_file.is_open()) {
        std::cout << "Failed to open file named : " << input_file_name << std::endl;
        return -1;
    }

    // 分配内存
    // measurement_pack_list：毫米波雷达/激光雷达实际测得的数据。数据包含测量值和时间戳，即融合算法的输入。
    // groundtruth_pack_list：每次测量时，障碍物位置的真值。对比融合算法输出和真值的差别，用于评估融合算法结果的好坏。
    std::vector<MeasurementPackage> measurement_pack_list;
    std::vector<GroundTruthPackage> groundtruth_pack_list;

    // 通过while循环将雷达测量值和真值全部读入内存，存入measurement_pack_list和groundtruth_pack_list中
    // Store radar & lidar data into memory
    std::string line;
    while (getline(input_file, line)) {
        std::string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        std::istringstream iss(line);
        long long timestamp;

        // 读取当前行的第一个元素，L代表Lidar数据，R代表Radar数据
        // Reads first element from the current line. L stands for Lidar. R stands for Radar.
        iss >> sensor_type;
        if (sensor_type.compare("L") == 0) {
            // 激光雷达数据 Lidar data
            // 该行第二个元素为测量值x，第三个元素为测量值y，第四个元素为时间戳(纳秒）
            // 2nd element is x; 3rd element is y; 4th element is timestamp(nano second)
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = Eigen::VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        } else if (sensor_type.compare("R") == 0) {
            // 毫米波雷达数据 Radar data
            // 该行第二个元素为距离pho，第三个元素为角度phi，第四个元素为径向速度pho_dot，第五个元素为时间戳(纳秒）
            // 2nd element is pho; 3rd element is phi; 4th element is pho_dot; 5th element is timestamp(nano second)
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = Eigen::VectorXd(3);
            float rho;
            float phi;
            float rho_dot;
            iss >> rho;
            iss >> phi;
            iss >> rho_dot;
            meas_package.raw_measurements_ << rho, phi, rho_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }

        // 当前行的最后四个元素分别是x方向上的距离真值，y方向上的距离真值，x方向上的速度真值，y方向上的速度真值
        // read ground truth data to compare later
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        gt_package.gt_values_ = Eigen::VectorXd(4);
        gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        groundtruth_pack_list.push_back(gt_package);
    }

    std::cout << "Success to load data." << std::endl;
    // 开始部署跟踪算法
    SensorFusion fuser;
    sensor_fusion::fusion_data msg;
    sensor_fusion::source_data msg2;

while (ros::ok()){
    for (size_t i = 0; i < measurement_pack_list.size(); ++i) {
       /***融合后的数据打印并且发布到ROS话题fusion_data中****/
        fuser.Process(measurement_pack_list[i]);
        Eigen::Vector4d x_out = fuser.kf_.GetX();
        msg.x=x_out(0);
        msg.y=x_out(1);
        msg.vx=x_out(2);
        msg.vy=x_out(3);
        sensor_info_pub.publish(msg);
        std::cout << "x " << x_out(0)
                  << " y " << x_out(1)
                  << " vx " << x_out(2)
                  << " vy " << x_out(3) 
                  << std::endl;
      /****这部分代码意图在将融合前的数据的发布到ROS话题source_data中*********/
       cout<<groundtruth_pack_list[i].gt_values_[0]<<endl;
       cout<<groundtruth_pack_list[i].gt_values_[1]<<endl;
       cout<<groundtruth_pack_list[i].gt_values_[2]<<endl;
       cout<<groundtruth_pack_list[i].gt_values_[3]<<endl;
       msg2.x=groundtruth_pack_list[i].gt_values_[0];
       msg2.y=groundtruth_pack_list[i].gt_values_[1];
       msg2.vx=groundtruth_pack_list[i].gt_values_[2];
       msg2.vy=groundtruth_pack_list[i].gt_values_[3];
       source_info_pub.publish(msg2);
}
      	
    }
//loop_rate.sleep();
}
