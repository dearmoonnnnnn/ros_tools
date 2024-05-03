/*
 * cloud_merging.cpp
 * 将同一个bag文件中的两个不同的点云数据进行合并
 * TODO: 配置文件，编译调试
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/point_cloud2_iterator.h>

ros::Publisher merged_pub;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

void callback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg, const sensor_msgs::PointCloud2ConstPtr& radar_msg) {
    sensor_msgs::PointCloud2 merged_cloud;
    sensor_msgs::PointCloud2Iterator<float> out_x(merged_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(merged_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(merged_cloud, "z");

    sensor_msgs::PointCloud2ConstIterator<float> in_x(*lidar_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*lidar_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*lidar_msg, "z");

    // 将LiDAR点云融合到merged_cloud中
    for (; in_x != lidar_msg->end(); ++in_x, ++in_y, ++in_z, ++out_x, ++out_y, ++out_z) {
        *out_x = *in_x;
        *out_y = *in_y;
        *out_z = *in_z;
    }

    sensor_msgs::PointCloud2ConstIterator<float> in_radar_x(*radar_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_radar_y(*radar_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_radar_z(*radar_msg, "z");

    // 将雷达点云融合到merged_cloud中
    for (; in_radar_x != radar_msg->end(); ++in_radar_x, ++in_radar_y, ++in_radar_z, ++out_x, ++out_y, ++out_z) {
        *out_x = *in_radar_x;
        *out_y = *in_radar_y;
        *out_z = *in_radar_z;
    }

    // 发布融合后的点云消息到radar_merged话题
    merged_cloud.header = lidar_msg->header;  // 使用LiDAR的时间戳和帧ID
    merged_pub.publish(merged_cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_merger");
    ros::NodeHandle nh;

    // 订阅两个点云话题
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/livox/lidar_PointCloud2", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub(nh, "ars548_process/detection_PointCloud2", 1);

    // 创建时间同步器，使用ApproximateTime策略
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), lidar_sub, radar_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // 创建发布者，发布融合后的点云消息到radar_merged话题
    merged_pub = nh.advertise<sensor_msgs::PointCloud2>("radar_merged", 1);

    ros::spin();

    return 0;
}
