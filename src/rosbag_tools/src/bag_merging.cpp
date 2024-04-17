#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "bag_merger");
    ros::NodeHandle nh;

    // 从ROS参数服务器获取输入bag文件的路径和输出bag文件的路径
    std::string bag1_path, bag2_path, output_bag_path;
    nh.param<std::string>("bag1_path", bag1_path, "/home/dearmoon/datasets/NWU/日晴不颠簸低速3/4DRadar/RiQingBuDianBoDiSu3.bag");   // Radar
    nh.param<std::string>("bag2_path", bag2_path, "/home/dearmoon/datasets/NWU/日晴不颠簸低速3/3.bag");                             // IMU
    nh.param<std::string>("output_bag_path", output_bag_path, "/home/dearmoon/datasets/NWU/日晴不颠簸低速3/imu_4DRadar.bag");

    // 打开输入bag文件和输出bag文件
    rosbag::Bag bag1(bag1_path, rosbag::bagmode::Read);
    rosbag::Bag bag2(bag2_path, rosbag::bagmode::Read);
    rosbag::Bag output_bag(output_bag_path, rosbag::bagmode::Write);

    // 创建一个Topic过滤器，选择需要保留的所有话题
    // rosbag::View view1(bag1, rosbag::TopicQuery("/a1") || rosbag::TopicQuery("/a2") || rosbag::TopicQuery("/a3"));
    // rosbag::View view2(bag2, rosbag::TopicQuery("/b1") || rosbag::TopicQuery("/b2"));
    rosbag::View view1(bag1, rosbag::TopicQuery("/ars548_process/detection_point_cloud"));
    rosbag::View view2(bag2, rosbag::TopicQuery("/livox/imu") );

    // 循环读取输入bag文件中的消息并写入输出bag文件
    for (const rosbag::MessageInstance& msg : view1) {
        output_bag.write(msg.getTopic(), msg.getTime(), msg);
    }

    for (const rosbag::MessageInstance& msg : view2) {
        std::cout << " msg.getTime(): " << msg.getTime() << std::endl;
        output_bag.write(msg.getTopic(), msg.getTime(), msg);
    }

    // 关闭输入和输出的bag文件
    bag1.close();
    bag2.close();
    output_bag.close();

    // 进入ROS事件循环，接收和处理ROS消息
    ros::spin();

    return 0;
}
