#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "command_sender");
    ros::NodeHandle nh;

    ros::Publisher command_pub = nh.advertise<std_msgs::String>("/command", 10);

    // 创建一个命令消息
    std_msgs::String command_msg;
    command_msg.data = "time";  // point_distribution, output_aftmapped

    // 等待发布者和订阅者连接
    ros::Duration(1.0).sleep();

    // 发布命令
    command_pub.publish(command_msg);
    ROS_INFO("Published command: %s", command_msg.data.c_str());

    ros::spinOnce();

    return 0;
}