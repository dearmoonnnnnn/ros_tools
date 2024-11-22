#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <rosbag_tools/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "point_xyzidv.h" // 自定义点类型


/*
 * 验证 sensor_msg::PointCloud2 数据字段
 */


void callback(const sensor_msgs::PointCloud2::ConstPtr& raw_msg){


    // 转换为 pcl::PointCloud<T> 后， 获取每个字段
    pcl::PointCloud<pcl::PointXYZIDV>::Ptr pcl_pointcloud( new pcl::PointCloud<pcl::PointXYZIDV>);
    pcl::fromROSMsg(*raw_msg, *pcl_pointcloud);
 
    if(1){
        for( size_t i = 0; i < pcl_pointcloud->size(); i++){
            float intensity = pcl_pointcloud->points[i].intensity;
            float doppler_velocity = pcl_pointcloud->points[i].doppler_velocity;   

            ROS_INFO("--------------- one point ---------------");
            ROS_INFO("After fromROSMsg , pcl_pointcloud, point[%zu] : intensity=%f, doppler velocity=%f", i, intensity, doppler_velocity);
        
        }
    }

    // 4DRadarSLAM 中的处理方法，同样转换为 pcl::PointCloud<T>
    if(0)
    {   
        auto radar_scan(new pcl::PointCloud<pcl::PointXYZIDV>);
        pcl::fromROSMsg (*raw_msg, *radar_scan);
        for (uint i = 0; i < radar_scan->size(); ++i)
        {
            const auto target = radar_scan->at(i);
            ROS_INFO("4DRadarSLAM method , radar_scan->at(%u) : intensity=%f, doppler velocity=%f", i, target.intensity, target.doppler_velocity);
        }
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_sensor_msg_PointCloud2");
    ros::NodeHandle nh;

    ros::Subscriber pointcloud_sub = nh.subscribe("/radar_merged", 10, callback);

    ros::spin();

    return 0;
}
