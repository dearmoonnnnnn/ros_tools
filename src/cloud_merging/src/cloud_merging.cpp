#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <cmath>            // For NAN and std::isnan
#include "point_xyzidv.h"


#include <sstream>  // For std::ostringstream
#include <iomanip>  // For std::fixed and std::setprecision
#include <string>

/*
 * step3:
 * cloud_merging.cpp
 * 将同一个bag文件中的两个不同的点云数据进行合并
 * 
 */

ros::Publisher merged_pub;
rosbag::Bag bag;
float distance_threshold = 0;  // 距离阈值，可以根据需要调整

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
cv::Mat Radar_to_Livox = (cv::Mat_<double>(4, 4) <<
 0.994838, 0.0187061, -0.0997379, -0.0379673, 
 -0.0209378, 0.999552, -0.0213763, -0.120289,
 0.0992934, 0.0233543, 0.994784, 0.41831,
 0,  0,  0,  1);

cv::Mat livox_to_Radar = (cv::Mat_<double>(4, 4) <<
 0.994838,  -0.0209378,  0.0992934, -0.0062827,
 0.0187061,  0.999552,   0.0233543,  0.11117599,
 -0.0997379, -0.0213763, 0.994784, -0.42248621,
  0,       0,        0,         1);       

void callback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg, const sensor_msgs::PointCloud2ConstPtr& radar_msg) {

    pcl::PointCloud<pcl::PointXYZIDV>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZIDV>);
    pcl::PointCloud<pcl::PointXYZIDV>::Ptr radar_cloud(new pcl::PointCloud<pcl::PointXYZIDV>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>);      // 激光雷达点云不包含多普勒速度，使用PointXYZI格式

    // 将两个点云数据转换为pcl::PointCloud<T>格式
    pcl::fromROSMsg(*lidar_msg, *lidar_cloud);
    pcl::fromROSMsg(*radar_msg, *radar_cloud);

    // 毫米波雷达数据
    for (int i = 0; i < radar_cloud->size(); i++) {
        pcl::PointXYZIDV point;
        point.x = radar_cloud->points[i].x;
        point.y = radar_cloud->points[i].y;
        point.z = radar_cloud->points[i].z;
        point.intensity = radar_cloud->points[i].intensity;
        point.doppler_velocity = radar_cloud->points[i].doppler_velocity;
        merged_cloud->push_back(point);
    }

    // 合并距离内的激光雷达数据
    int lidar_count = 0;             // 激光雷达计数器，填充的激光雷达点数量
    for (const auto& lidar_point : lidar_cloud->points) {
        
        // 应用变换矩阵进行坐标变换，将点从 livox 坐标系转到 Radar 坐标系
        cv::Mat ptMat, dstMat; 
        ptMat = (cv::Mat_<double>(4, 1) << lidar_point.x, lidar_point.y, lidar_point.z, 1);    
        // Perform matrix multiplication and save as Mat_ for easy element access
        dstMat= livox_to_Radar * ptMat;
        
        pcl::PointXYZIDV point;
        point.x = dstMat.at<double>(0,0);
        point.y = dstMat.at<double>(1,0);
        point.z = dstMat.at<double>(2,0);
        point.intensity = lidar_point.intensity;
        point.doppler_velocity = NAN;   // 激光雷达没有多普勒速度

        for (const auto& radar_point : radar_cloud->points) {
            float dx = point.x - radar_point.x;
            float dy = point.y - radar_point.y;
            float dz = point.z - radar_point.z;
            float distance = std::sqrt(dx * dx + dy * dy + dz * dz);


            if (distance < distance_threshold) {
                merged_cloud->push_back(point);
                lidar_count++;
                break; // 只保留一次，避免重复加入激光雷达点
            }
        }
    }


    ROS_INFO("lidar origin size: %ld", lidar_cloud->size());
    ROS_INFO("lidar pushed size: %d", lidar_count);
    ROS_INFO("radar size: %ld", radar_cloud->size());
    ROS_INFO("merged size: %ld", merged_cloud->size());
    ROS_INFO("/---------------------------------------");
    /**************************融合点云数据***************************/


    // 转换为sensor_msgs::PointCloud2格式
    sensor_msgs::PointCloud2 merged_msg;
    pcl::toROSMsg(*merged_cloud, merged_msg);
    merged_msg.header.stamp = radar_msg->header.stamp;
    merged_msg.header.frame_id = "radar";

    // toROSMsg会自动将size填充到width
    // ROS_INFO("merged_msg.width = %d; merged_cloud.size = %ld ", merged_msg.width, merged_cloud->size());

    // merged_pub.publish(merged_msg);
    bag.write("/radar_merged", radar_msg->header.stamp, merged_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_merging");
    ros::NodeHandle nh;

    /**** 根据 distance_threshold，命名输出 bag 文件，只保留整数部分 ****/
    // std::string bag_path = "/home/dearmoon/datasets/NWU/日晴不颠簸低速3/enhancing/";
    std::string output_bag_path = nh.param<std::string>("/cloud_merging/output_bag_path", "/home/dearmoon/datasets/NWU/2222222/enhancing/");  
    ROS_INFO("output_bag_path : %s", output_bag_path.c_str());
    std::ostringstream output_bag;
    output_bag << output_bag_path << "radar_lidar_output_" << std::fixed << std::setprecision(0) << distance_threshold << ".bag" ;
    std::string output_str = output_bag.str();

    // 创建发布者，发布融合后的点云消息到radar_merged话题
    merged_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_merged", 1);

    bag.open(output_str, rosbag::bagmode::Write);

    // 订阅两个点云话题
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/livox/lidar_PointCloud2", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub(nh, "/ars548_process/detectioin_PointCloud2", 10);

    // 创建时间同步器，使用ApproximateTime策略
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(32), lidar_sub, radar_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
   // sync.setMaxIntervalDuration(ros::Duration(0.05));       // 设置最大时间间隔，单位秒

    ros::spin();

    bag.close();

    return 0;
}
