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

bool flag = true;

void callback(const sensor_msgs::PointCloud2::ConstPtr& raw_msg){

    // 检查 sensor_msgs::PointCloud2 字段定义
    if (0){

        std::cout << "--------------- 开始：检查字段定义 --------------- " << std::endl;
        for (const auto& field : raw_msg->fields) {
            ROS_INFO("Field name: %s, offset: %u, datatype: %u, count: %u", 
            field.name.c_str(), field.offset, field.datatype, field.count);
        }
        std::cout << "--------------- 结束：检查字段定义 --------------- " << std::endl;
    }


    // 转换为 pcl::PointCloud<T> 后， 获取每个字段。
    // 所有字段解析正确
    if (1){
        std::cout << "--------------- 开始：转换为 pcl::PointCloud<T> 后解析 --------------- " << std::endl;
        pcl::PointCloud<pcl::PointXYZIDV>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZIDV>);

        // 从ROS消息转换到PCL点云
        pcl::fromROSMsg(*raw_msg, *pcl_pointcloud);

        // 遍历点云，打印 intensity 和 doppler_velocity
        for (size_t i = 0; i < pcl_pointcloud->size(); i++) {
            float intensity = pcl_pointcloud->points[i].intensity;
            float doppler_velocity = pcl_pointcloud->points[i].doppler_velocity;

            ROS_INFO("--------------- one point ---------------");
            ROS_INFO("After fromROSMsg , pcl_pointcloud, point[%zu] : intensity=%f, doppler velocity=%f", i, intensity, doppler_velocity);
        }


        std::cout << "--------------- 结束：转换为 pcl::PointCloud<T> 后解析 --------------- " << std::endl;

        // // 获取字段列表
        // std::string fields_list;
        // pcl::getFieldsList(*pcl_pointcloud, fields_list); 

        // // 打印字段列表
        // ROS_INFO("PCL fields: %s", fields_list.c_str());

        // flag = false;
    }

    // 4DRadarSLAM 中的解析方法，同样转换为 pcl::PointCloud<T>
    // 所有字段解析正确
    if (0)
    {   
        auto radar_scan(new pcl::PointCloud<pcl::PointXYZIDV>);
        pcl::fromROSMsg (*raw_msg, *radar_scan);
        for (uint i = 0; i < radar_scan->size(); ++i)
        {
            const auto target = radar_scan->at(i);
            ROS_INFO("4DRadarSLAM method , radar_scan->at(%u) : intensity=%f, doppler velocity=%f", i, target.intensity, target.doppler_velocity);
        }
        // flag = false;
    }

    // 手动解析 pointcloud2 字段值
    // 结果：所有字段解析正确
    if (0){    
        std::cout << "--------------- 开始：手动解析 pointcloud2 字段值 --------------- " << std::endl;
        const uint8_t* data_ptr = raw_msg->data.data();
        for (size_t i = 0; i < raw_msg->width; ++i) {
            float x, y, z, intensity, doppler_velocity;
            memcpy(&x, data_ptr + raw_msg->fields[0].offset, sizeof(float));
            memcpy(&y, data_ptr + raw_msg->fields[1].offset, sizeof(float));
            memcpy(&z, data_ptr + raw_msg->fields[2].offset, sizeof(float));
            memcpy(&intensity, data_ptr + raw_msg->fields[3].offset, sizeof(float));
            memcpy(&doppler_velocity, data_ptr + raw_msg->fields[4].offset, sizeof(float));

            ROS_INFO("Point %zu: x=%f, y=%f, z=%f, intensity=%f, doppler_velocity=%f", 
                    i, x, y, z, intensity, doppler_velocity);

            data_ptr += raw_msg->point_step;  // 跳到下一个点
        }

        std::cout << "--------------- 结束：手动解析 pointcloud2 字段值 --------------- " << std::endl;

    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_sensor_msg_PointCloud2");
    ros::NodeHandle nh;

    // ros::Subscriber pointcloud_sub = nh.subscribe("/radar_merged", 10, callback);
    ros::Subscriber pointcloud_sub = nh.subscribe("/eagle_data/pc2_raw", 10, callback);

    ros::spin();

    return 0;
}
