#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <rosbag_tools/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

ros::Publisher pcl2_pub;
ros::Publisher custom_pcl2_pub;
rosbag::Bag output_bag;

// 回调函数处理 sensor_msgs::PointCloud 消息
void pointcloudCallback(const sensor_msgs::PointCloudConstPtr& msg) {

    pcl::PointXYZI point_xyzi;
    pcl::PointCloud<PointXYZI>::Ptr radarcloud_xyzi( new pcl::PointCloud<PointXYZI> );

    // radarcloud_xyzi->header.frame_id = baselinkFrame;
    radarcloud_xyzi->header.seq = eagle_msg->header.seq;        // 序列号，用于标识消息的顺序
    radarcloud_xyzi->header.stamp = eagle_msg->header.stamp.toSec() * 1e6;

    for(int i = 0; i < msg->points.size(); i++){
        point_xyzi.x = msg->points[i].x;
        point_xyzi.y = msg->points[i].y;
        point_xyzi.z = msg->points[i].z;
        point_xyzi.intensity = msg->points[i].channels[2].value[i];
        radarcloud_xyzi->points.push_back(point_xyzi);    
    }

    // 将 sensor_msgs::PointCloud 转换为 sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 pcl2_msg;
    pcl::toROSMsg(*radar_cloud_xyzi, pcl2_msg);

    // 发布转换后的 sensor_msgs::PointCloud2 消息
    // pcl2_pub.publish(pcl2_msg);

    // 将转换后的消息写入新的 ROS bag 文件
    output_bag.write("/ars458_process/detectioin_point_cloud_converted", pcl2_msg.header.stamp, pcl2_msg);
    std::cout << "radar timestamp" << pcl2_msg.header.stamp << std::endl;
}

// 回调函数处理 livox_ros_driver::CustomMsg 消息
void customMsgCallback(const rosbag_tools::CustomMsgConstPtr& msg) {
    // 将 livox_ros_driver::CustomMsg 转换为 sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 custom_pcl2_msg;
    pcl::toROSMsg(*msg, custom_pcl2_msg);

    // 发布转换后的 sensor_msgs::PointCloud2 消息
    custom_pcl2_pub.publish(custom_pcl2_msg);

    // 将转换后的消息写入新的 ROS bag 文件
    output_bag.write("/livox/lidar_converted", custom_pcl2_msg.header.stamp, custom_pcl2_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "convert_messages_to_pointcloud2");
    ros::NodeHandle nh;

    // 创建 ROS 发布器
    pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("/ars458_process/detectioin_point_cloud_converted", 10);
    custom_pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_converted", 10);

    // 打开新的 ROS bag 文件
    output_bag.open("/home/dearmoon/datasets/NWU/日晴不颠簸低速3/enhancing/radar_lidar_step2.bag", rosbag::bagmode::Write);

    // 订阅 sensor_msgs::PointCloud 和 livox_ros_driver::CustomMsg 话题，并设置回调函数
    ros::Subscriber pointcloud_sub = nh.subscribe("/ars458_process/detectioin_point_cloud", 10, pointcloudCallback);
    ros::Subscriber custom_sub = nh.subscribe("/livox/lidar", 10, customMsgCallback);

    ros::spin();

    // 关闭 ROS bag 文件
    output_bag.close();

    return 0;
}
