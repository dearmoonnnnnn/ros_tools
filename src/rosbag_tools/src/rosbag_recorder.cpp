#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Imu.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

rosbag::Bag bag; // 声明一个rosbag
int cloud_msg_count = 0;
int imu_msg_count = 0;

int cloud_msg_count_max = 9918;

void cloud_callback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    // 在这里处理接收到的PointCloud消息
    // 可以通过msg来访问消息的数据

    std::cout << "/------- cloud_callback" << std::endl;

    if (bag.isOpen()) {
        bag.write("/ars548_process/detection_point_cloud", msg->header.stamp, *msg);
        cloud_msg_count++;
        std::cout << "radar timestamp : " << msg->header.stamp << std::endl;
        std::cout << "cloud_msg_count : " << cloud_msg_count << std::endl;

        // 打印第一个点的信号强度，调试输出
        // std::cout << "intensity : " << msg->channel[1].value[0] << std::endl;

        if (cloud_msg_count >= cloud_msg_count_max) {
            ROS_INFO("Stopping node as cloud_msg_count reached %d", cloud_msg_count_max);
            bag.close();                    // 关闭Bag文件
            ros::shutdown();
        }
    }
    
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    
    std::cout << "imu_callback" << std::endl;
    if (bag.isOpen()) {
        bag.write("/livox/imu", msg->header.stamp, *msg);
        imu_msg_count++;
        std::cout << "imu timestamp :" << msg->header.stamp << std::endl;
        std::cout << "imu_msg_count : " << imu_msg_count << std::endl;
    }
}
 
int main(int argc, char **argv) {
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle nh;

    // 打开Bag文件以写入
    bag.open("/home/dearmoon/datasets/NWU/夜雪不颠簸高速/enhancing/yxbdbgs.bag",  rosbag::bagmode::Write);
    // 如果 cloud_msg_count 达到阈值，停止运行节点

    ros::Subscriber sub_radar = nh.subscribe<sensor_msgs::PointCloud>("/ars548_process/detection_point_cloud", 10, cloud_callback);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/livox/imu", 10, imu_callback);

    ros::spin(); // 进入ROS主循环，等待消息

    bag.close(); // 关闭Bag文件

    return 0;
}
