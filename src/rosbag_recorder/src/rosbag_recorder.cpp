#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

rosbag::Bag bag; // 声明一个rosbag

void alCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    // 在这里处理接收到的PointCloud消息
    // 可以通过msg来访问消息的数据

    if (bag.isOpen()) {
        bag.write("/ars548_process/detection_point_cloud", msg->header.stamp, *msg);
        std::cout << "msg->header.stamp : " << msg->header.stamp << std::endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle nh;

    // 打开Bag文件以写入
    bag.open("/home/dearmoon/datasets/NWU/日晴半室内低速2/4DRadar/RiQingBanShiNeiDiSu3.bag", rosbag::bagmode::Write);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud>("/ars548_process/detection_point_cloud", 10, alCallback);

    ros::spin(); // 进入ROS主循环，等待消息

    bag.close(); // 关闭Bag文件

    return 0;
}
