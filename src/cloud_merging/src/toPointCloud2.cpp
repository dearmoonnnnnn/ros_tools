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
 * step2:
 * 将 sensor_msgs::PointCloud 消息转换为 sensor_msgs::PointCloud2 消息
 * 将 CustomMsg 消息转换为 sensor_msgs::PointCloud2 消息
 * pcl::PointCloud<T> 使用自定义的pcl点类型，同时包含位置信息、信号强度和多普勒速度
 * 
 * 输入：订阅的两个话题
 * 输出：保存两个话题的 bag 文件
 */


// ros::Publisher pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("/ars458_process/detectioin_point_cloud", 10);
// ros::Publisher custom_pcl2_pub = custom_pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar", 10);
rosbag::Bag output_bag;

/* 
 * 回调函数处理 sensor_msgs::PointCloud 消息
 * 将 sensor_msgs::PointCloud 消息转换为 sensor_msgs::PointCloud2 消息
 * 点包含信号强度和多普勒速度信息
 */
void pointcloudCallback(const sensor_msgs::PointCloudConstPtr& msg) {

    sensor_msgs::PointCloud2 pcl2_msg;

    // 使用PointXYZI
    // pcl::PointXYZI point_pcl;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr radarCloud_pcl( new pcl::PointCloud<pcl::PointXYZI> );

    // 使用自定义点 PointXYZIDV
    pcl::PointXYZIDV point_pcl;
    pcl::PointCloud<pcl::PointXYZIDV>::Ptr radarCloud_pcl( new pcl::PointCloud<pcl::PointXYZIDV>);

    radarCloud_pcl->header.frame_id = "radar";
    radarCloud_pcl->header.seq = msg->header.seq;        // 序列号，用于标识消息的顺序
    radarCloud_pcl->header.stamp = msg->header.stamp.toSec() * 1e6;
 
    for(int i = 0; i < msg->points.size(); i++){
        point_pcl.x = msg->points[i].x;
        point_pcl.y = msg->points[i].y;
        point_pcl.z = msg->points[i].z;
        point_pcl.intensity = msg->channels[1].values[i];        // 添加强度信息
        
        point_pcl.doppler_velocity = msg->channels[0].values[i]; // 多普勒速度信息
        radarCloud_pcl->points.push_back(point_pcl);    
    }
 
    // 将 pcl::PointCloud 转换为 sensor_msgs::PointCloud2
    pcl::toROSMsg(*radarCloud_pcl, pcl2_msg);

    pcl2_msg.header.stamp = msg->header.stamp; 
    pcl2_msg.header.frame_id = "radar";
    pcl2_msg.header.seq = msg->header.seq;
    pcl2_msg.width = radarCloud_pcl->points.size();
    pcl2_msg.height = 1;


    /*
     * sensor_msgs::PointCloud2强度信息调试输出
     */

    if(1){
        int intensity_offset = -1;
        for (const auto& field : pcl2_msg.fields)
        {
            if (field.name == "intensity")
            {
                intensity_offset = field.offset;
                break;
            }
        }

        if (intensity_offset == -1)
        {
            ROS_ERROR("No intensity field found in PointCloud2 message.");
            return;
        }

        // 计算每个点的步长
        int point_step = pcl2_msg.point_step;

        // 遍历每个点，提取强度信息
        for (size_t i = 0; i < pcl2_msg.width * pcl2_msg.height; ++i)
        {
            const uint8_t* point_data = &pcl2_msg.data[i * point_step];
            float intensity;
            memcpy(&intensity, point_data + intensity_offset, sizeof(float));

            // 打印或处理强度信息
            ROS_INFO("Point %zu: intensity=%f", i, intensity);
        }
    }

    /*
     * sensor_msgs::PointCloud2多普勒速度调试输出
     */
    if(0){
        int doppler_offset = -1;
        for (const auto& field : pcl2_msg.fields)
        {
            if (field.name == "doppler_velocity")
            {
                doppler_offset = field.offset;
                break;
            }
        }


        if (doppler_offset == -1)
        {
            ROS_ERROR("No doppler velocity field found in PointCloud2 message.");
            return;
        }

        // 计算每个点的步长
        int point_step = pcl2_msg.point_step;

        // 遍历每个点，提取强度信息
        for (size_t i = 0; i < pcl2_msg.width * pcl2_msg.height; ++i)
        {
            const uint8_t* point_data = &pcl2_msg.data[i * point_step];
            float doppler_velocity;
            memcpy(&doppler_velocity, point_data + doppler_offset, sizeof(float));

            // 打印或处理强度信息
            ROS_INFO("Point %zu: doppler velocity=%f", i, doppler_velocity);
        }
    }
    
    /*
     * fromROSMsg调试
     */
    if(1){
        pcl::PointCloud<pcl::PointXYZIDV>::Ptr pointcloud_test( new pcl::PointCloud<pcl::PointXYZIDV>);
        pcl::fromROSMsg(pcl2_msg, *pointcloud_test);
 
        for( size_t i = 0; i < pointcloud_test->size(); i++){
            float intensity = pointcloud_test->points[i].intensity;
            float doppler_velocity = pointcloud_test->points[i].doppler_velocity;   

            ROS_INFO("After fromROSMsg , pointcloud_test, point[%zu] : intensity=%f, doppler velocity=%f", i, intensity, doppler_velocity);
        }
    }

    // 将转换后的消息写入新的 ROS bag 文件
    // output_bag.write("/ars548_process/detection_PointCloud2", pcl2_msg.header.stamp, pcl2_msg);

    // 若要插值，使用上面的话题
    output_bag.write("/filtered_points", pcl2_msg.header.stamp, pcl2_msg);
    // pcl2_pub.publish(pcl2_msg);

    // ROS_INFO("radar timestamp: %f", pcl2_msg->header.stamp.toSec());
    // std::cout << "radar timestamp: " << pcl2_msg->header.stamp << std::endl;

}

/*
 * 回调函数处理 livox_ros_driver::CustomMsg 消息
 * 将 livox_ros_driver::CustomMsg 消息转换为 sensor_msgs::PointCloud2 消息
 * 点信号强度，不包含多普勒速度信息
 */
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
    output_bag.write("/livox/lidar", custom_pcl2_msg.header.stamp, custom_pcl2_msg);


    ROS_INFO("lidar timestamp: %f", custom_pcl2_msg.header.stamp.toSec());
    std::cout << "lidar timestamp: " << custom_pcl2_msg.header.stamp << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "convert_messages_to_pointcloud2");
    ros::NodeHandle nh;

    // 创建 ROS 发布器


    // 打开新的 ROS bag 文件
    output_bag.open("/home/dearmoon/datasets/NWU/夜雪不颠簸高速/radar_input.bag", rosbag::bagmode::Write);
    // output_bag.open("/home/dearmoon/datasets/NWU/日晴不颠簸低速3/enhancing/one_msg_test_step2.bag", rosbag::bagmode::Write);

    // 订阅 sensor_msgs::PointCloud 和 livox_ros_driver::CustomMsg 话题，并设置回调函数
    ros::Subscriber pointcloud_sub = nh.subscribe("/ars548_detectionlist", 10, pointcloudCallback);
    ros::Subscriber custom_sub = nh.subscribe("/livox/lidar", 10, customMsgCallback);

    ros::spin();

    // 关闭 ROS bag 文件
    output_bag.close();

    return 0;
}
