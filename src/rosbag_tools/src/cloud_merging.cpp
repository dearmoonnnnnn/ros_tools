#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

/*
* 订阅两个话题，将两个话题的点云进行融合，发布融合后的话题
* TODO: 增加标定矩阵，将点云进行坐标变换
*/


// 全局变量
sensor_msgs::PointCloud2ConstPtr detection_msg;
ros::Time prev_msg_time;
ros::Publisher pub;


// void livoxCallback(const livox_ros_driver::CustomMsgConstPtr& msg)
// {
//     // 将CustomMsg转换为PointCloud2
//     sensor_msgs::PointCloud2::ConstPtr cloud_msg = convertToPointCloud2(msg);

//     // 存储上一个消息的时间戳
//     if (prev_msg_time.isZero())
//         prev_msg_time = msg->header.stamp;
    
//     // 等待另一个话题的消息
//     ros::Rate rate(10);
//     while (ros::ok() && detection_msg == nullptr && rate.sleep())
//     {
//         ros::spinOnce();
//     }

//     if (detection_msg != nullptr && detection_msg->header.stamp - prev_msg_time < ros::Duration(0.1))
//     {
//         // 融合两个消息
//         sensor_msgs::PointCloud2 fused_msg = *cloud_msg;
//         appendPointCloud(*detection_msg, fused_msg);

//         // 发布融合后的消息
//         pub.publish(fused_msg);
//         ROS_INFO("Messages fused and published.");
//     }
//     else
//     {
//         // 如果时间戳差太大，直接发布livox的消息
//         pub.publish(*cloud_msg);
//     }

//     // 更新上一个消息的时间戳
//     prev_msg_time = msg->header.stamp;
// }
            
void livoxCallback(const livox_ros_driver::CustomMsgConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud_msg = convertToPointCloud2(msg);

    if (prev_msg_time.isZero()) {
        prev_msg_time = msg->header.stamp;
    }

    if (detection_msg && abs((detection_msg->header.stamp - msg->header.stamp).toSec()) < 0.1)
    {
        sensor_msgs::PointCloud2 fused_msg = cloud_msg;
        appendPointCloud(*detection_msg, fused_msg);
        pub.publish(fused_msg);
        ROS_INFO("Messages fused and published.");
    } else {
        pub.publish(cloud_msg);
    }

    prev_msg_time = msg->header.stamp;
}

void detectionCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    detection_msg = msg;
}

// 将CustomMsg转换为PointCloud2
// sensor_msgs::PointCloud2::ConstPtr convertToPointCloud2(const livox_ros_driver::CustomMsgConstPtr& msg)
// {
//     sensor_msgs::PointCloud2::ConstPtr cloud_msg(new sensor_msgs::PointCloud2);
//     cloud_msg->header = msg->header;
//     // 将点云数据复制到sensor_msgs::PointCloud2中
//     // ... 您可以在这里实现将CustomMsg转换为PointCloud2的逻辑

//     // 其余字段可以设置为默认值
//     cloud_msg->height = 1;
//     cloud_msg->width = static_cast<uint32_t>(msg->points.size());
//     cloud_msg->is_dense = false;
//     return cloud_msg;
// }

sensor_msgs::PointCloud2 convertToPointCloud2(const livox_ros_driver::CustomMsgConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    // 填充pcl_cloud
    for (const auto& pt : msg->points) {
        pcl::PointXYZI point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = pt.z;
        point.intensity = pt.reflectivity;
        pcl_cloud.push_back(point);
    }
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header = msg->header;
    return cloud_msg;
}

// 在点云消息中添加另一个点云
// void appendPointCloud(const sensor_msgs::PointCloud2& src, sensor_msgs::PointCloud2& dst)
// {
//     // 确保两个消息的字段一致
//     if (src.height != dst.height || src.width != dst.width || src.is_dense != dst.is_dense)
//     {
//         ROS_ERROR("Inconsistent point cloud fields.");
//         return;
//     }

//     // 将src中的点添加到dst中
//     for (size_t i = 0; i < src.data.size() / 4; ++i)
//     {
//         dst.data.push_back(src.data[i * 4 + 0]); // x
//         dst.data.push_back(src.data[i * 4 + 1]); // y
//         dst.data.push_back(src.data[i * 4 + 2]); // z
//         dst.data.push_back(src.data[i * 4 + 3]); // intensity/color
//     }
// }

// 在点云消息中添加另一个点云
void appendPointCloud(const sensor_msgs::PointCloud2& src, sensor_msgs::PointCloud2& dst)
{
    pcl::PointCloud<pcl::PointXYZI> pcl_src, pcl_dst;
    pcl::fromROSMsg(src, pcl_src);
    pcl::fromROSMsg(dst, pcl_dst);

    pcl_dst += pcl_src;
    pcl::toROSMsg(pcl_dst, dst);
    dst.header = src.header;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_fuser");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("/livox/lidar", 10, livoxCallback);
    ros::Subscriber sub2 = nh.subscribe("/ars548_process/detection_point_cloud", 10, detectionCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/ars548_process/detection_point_cloud_fused", 10);

    ros::spin();
    return 0;
}