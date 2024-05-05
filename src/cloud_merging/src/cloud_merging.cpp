/*
 * cloud_merging.cpp
 * 将同一个bag文件中的两个不同的点云数据进行合并
 * TODO: 配置文件，编译调试
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>


ros::Publisher merged_pub;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
cv::Mat livox_to_Radar = (cv::Mat_<double>(4, 4) <<
 0.994838, 0.0187061, -0.0997379, -0.0379673, 
 -0.0209378, 0.999552, -0.0213763, -0.120289,
 0.0992934, 0.0233543, 0.994784, 0.41831,
 0,  0,  0,  1);

void callback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg, const sensor_msgs::PointCloud2ConstPtr& radar_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr radar_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 将两个点云数据转换为pcl::PointCloud<pcl::PointXYZI>格式
    pcl::fromROSMsg(*lidar_msg, *lidar_cloud);
    pcl::fromROSMsg(*radar_msg, *radar_cloud);

    // 融合点云数据
    // 应用变换矩阵进行坐标变换 
    for (int i = 0; i < lidar_cloud->size(); i++) {
        // 将点从 livox 坐标系转到 Radar 坐标系
        cv::Mat ptMat, dstMat; 
        ptMat = (cv::Mat_<double>(4, 1) << lidar_cloud->points[i].x, lidar_cloud->points[i].y, lidar_cloud->points[i].z, 1);    
        // Perform matrix multiplication and save as Mat_ for easy element access
        dstMat= livox_to_Radar * ptMat;
        
        pcl::PointXYZI point;
        point.x = dstMat.at<double>(0,0);
        point.y = dstMat.at<double>(1,0);
        point.z = dstMat.at<double>(2,0);
        point.intensity = lidar_cloud->points[i].intensity;
        merged_cloud->push_back(point);
    }

    // 将点从 livox 坐标系转到 Radar 坐标系
    for (int i = 0; i < radar_cloud->size(); i++) {
        pcl::PointXYZI point;
        point.x = radar_cloud->points[i].x;
        point.y = radar_cloud->points[i].y;
        point.z = radar_cloud->points[i].z;
        point.intensity = radar_cloud->points[i].intensity;
        merged_cloud->push_back(point);
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_merger");
    ros::NodeHandle nh;

    // 订阅两个点云话题
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/livox/lidar_PointCloud2", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub(nh, "ars548_process/detection_PointCloud2", 10);

    // 创建时间同步器，使用ApproximateTime策略
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), lidar_sub, radar_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // 创建发布者，发布融合后的点云消息到radar_merged话题
    merged_pub = nh.advertise<sensor_msgs::PointCloud2>("radar_merged", 1);

    ros::spin();

    return 0;
}
