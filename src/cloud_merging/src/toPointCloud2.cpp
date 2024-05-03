#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <rosbag_tools/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/*
 * 将 sensor_msgs::PointCloud 消息转换为 sensor_msgs::PointCloud2 消息
 * 将 CustomMsg 消息转换为 sensor_msgs::PointCloud2 消息
 */


ros::Publisher pcl2_pub;
ros::Publisher custom_pcl2_pub;
rosbag::Bag output_bag;

// 回调函数处理 sensor_msgs::PointCloud 消息
void pointcloudCallback(const sensor_msgs::PointCloudConstPtr& msg) {

    pcl::PointXYZI point_xyzi;
    pcl::PointCloud<pcl::PointXYZI>::Ptr radarCloud_xyzi( new pcl::PointCloud<pcl::PointXYZI> );

    radarCloud_xyzi->header.frame_id = "radar";
    radarCloud_xyzi->header.seq = msg->header.seq;        // 序列号，用于标识消息的顺序
    radarCloud_xyzi->header.stamp = msg->header.stamp.toSec() * 1e6;

    for(int i = 0; i < msg->points.size(); i++){
        point_xyzi.x = msg->points[i].x;
        point_xyzi.y = msg->points[i].y;
        point_xyzi.z = msg->points[i].z;
        point_xyzi.intensity = msg->channels[2].values[i];   // 添加强度信息
        radarCloud_xyzi->points.push_back(point_xyzi);    
    }

    // 将 sensor_msgs::PointCloud 转换为 sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 pcl2_msg;
    pcl::toROSMsg(*radarCloud_xyzi, pcl2_msg);

    pcl2_msg.header.stamp = msg->header.stamp; 
    pcl2_msg.header.frame_id = "radar";


    // 发布转换后的 sensor_msgs::PointCloud2 消息
    // pcl2_pub.publish(pcl2_msg);

    // 将转换后的消息写入新的 ROS bag 文件
    output_bag.write("/ars458_process/detectioin_PointCloud2", pcl2_msg.header.stamp, pcl2_msg);
    std::cout << "radar timestamp" << pcl2_msg.header.stamp << std::endl;
}

// 回调函数处理 livox_ros_driver::CustomMsg 消息
void customMsgCallback(const rosbag_tools::CustomMsgConstPtr& msg) {
    pcl::PointXYZI point_xyzi;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarCloud_xyzi( new pcl::PointCloud<pcl::PointXYZI> );

    lidarCloud_xyzi->header.frame_id = "livox";
    lidarCloud_xyzi->header.seq = msg->header.seq;        // 序列号，用于标识消息的顺序
    lidarCloud_xyzi->header.stamp = msg->header.stamp.toSec() * 1e6;


    for(int i = 0; i < msg->points.size(); i++){
        point_xyzi.x = msg->points[i].x;
        point_xyzi.y = msg->points[i].y;
        point_xyzi.z = msg->points[i].z;
        point_xyzi.intensity = msg->points[i].reflectivity;
        lidarCloud_xyzi->points.push_back(point_xyzi);    
    }

    // 将 livox_ros_driver::CustomMsg 转换为 sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 custom_pcl2_msg;
    pcl::toROSMsg(*lidarCloud_xyzi, custom_pcl2_msg);

    custom_pcl2_msg.header.stamp = msg->header.stamp; 
    custom_pcl2_msg.header.frame_id = "lidar";
    // 发布转换后的 sensor_msgs::PointCloud2 消息
    // custom_pcl2_pub.publish(custom_pcl2_msg);

    // 将转换后的消息写入新的 ROS bag 文件
    output_bag.write("/livox/lidar_PointCloud2", custom_pcl2_msg.header.stamp, custom_pcl2_msg);
    std::cout << "lidar timestamp" << custom_pcl2_msg.header.stamp << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "convert_messages_to_pointcloud2");
    ros::NodeHandle nh;

    // 创建 ROS 发布器
    pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("/ars458_process/detectioin_point_cloud", 10);
    custom_pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar", 10);

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
