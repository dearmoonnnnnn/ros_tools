#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

// 全局发布器
ros::Publisher pub_cloud2;
ros::Publisher pub_transformed_cloud1;

Eigen::Matrix4f transform; // 转换矩阵

// 点云转换回调函数
void cloud1Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    ROS_INFO("cloud1Callback called");
    // 输入点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud_in);

    // 转换后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_in, *transformed_cloud, transform);

    // 发布转换后的点云
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*transformed_cloud, output);
    output.header.frame_id = "map";
    pub_transformed_cloud1.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_transformer");
    ros::NodeHandle nh;

    // 初始化转换矩阵 (Radar_to_Livox)
    cv::Mat Radar_to_Livox = (cv::Mat_<double>(4, 4) <<
        0.994838, 0.0187061, -0.0997379, -0.0379673,
        -0.0209378, 0.999552, -0.0213763, -0.120289,
        0.0992934, 0.0233543, 0.994784, 0.41831,
        0, 0, 0, 1);

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            transform(i, j) = static_cast<float>(Radar_to_Livox.at<double>(i, j));
        }
    }

    // 订阅原始点云1话题
    ros::Subscriber sub_cloud1 = nh.subscribe("/cloud1", 1, cloud1Callback);

    // 发布转换后的点云1
    pub_transformed_cloud1 = nh.advertise<sensor_msgs::PointCloud2>("/transformed_cloud1", 1);

    // 发布原始点云2（这里直接重映射即可，无需处理）
    pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud2", 1);

    ros::spin();
    return 0;
}
