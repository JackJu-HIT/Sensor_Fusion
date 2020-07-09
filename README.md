# Sensor_Fusion
ROS下的一个激光雷达和毫米波雷达的数据融合算法
主要使用了卡尔曼过滤算法和扩展卡尔曼滤波算法。\
# 发布的数据格式：
Topic：/ fusion_data融合后数据
Topic：/ orgin_data融合前的数据
# 融合思路
这里采用的是异步融合，假设上一时刻用的激光雷达测量的数据，此时刻用的是毫米波雷达的数据。
# 运行
你需要安装ROS环境
ubuntu下cmake编译：编译后会生成SensorFusion执行文件，直接运行：./SensorFusion
