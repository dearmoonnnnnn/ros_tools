#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rosbag_tools/CustomMsg.h>

ros::Publisher pub;
static int radar_frame_count = 0; // 添加静态变量记录帧序号
static int lidar_frame_count = 0;

void radarcloud_Callback(const sensor_msgs::PointCloud::ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width = cloud_msg->points.size();
    cloud.height = 1;
    cloud.is_dense = true; // 初始化为 true

    // 将 sensor_msgs::PointCloud 转换为 pcl::PointCloud<pcl::PointXYZ>
    for (size_t i = 0; i < cloud_msg->points.size(); ++i)
    {
        pcl::PointXYZI point;
        point.x = cloud_msg->points[i].x;
        point.y = cloud_msg->points[i].y;
        point.z = cloud_msg->points[i].z;
        point.intensity = cloud_msg->channels[1].values[i];

        // 检查是否有无效点
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            cloud.is_dense = false; // 如果发现无效点,将 is_dense 设置为 false
        }

        cloud.points.push_back(point);
    }

    // 根据当前帧序号动态生成输出文件名
    std::string output_file = "/home/dearmoon/datasets/NWU/日晴不颠簸高速4/radar_pcd/radar_pcd_" + std::to_string(radar_frame_count) + ".pcd";

    pcl::io::savePCDFileASCII(output_file, cloud);
    ROS_INFO("Saved %lu data points to %s.", cloud.points.size(), output_file.c_str());

    radar_frame_count++; // 增加帧序号
}

void lidarcloud_Callback(const rosbag_tools::CustomMsg & cloud_msg)
{
    // 创建PointCloud对象
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    
    // 设置PointCloud对象的属性
    pcl_cloud.width = cloud_msg.point_num;
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = false; // 数据可能有缺失点

    // 遍历CustomMsg中的每个点并添加到PointCloud中
    for (size_t i = 0; i < cloud_msg.point_num; ++i)
    {
        pcl::PointXYZI point;
        point.x = cloud_msg.points[i].x;
        point.y = cloud_msg.points[i].y;
        point.z = cloud_msg.points[i].z;
        point.intensity = static_cast<float>(cloud_msg.points[i].reflectivity) / 255.0f; // 归一化到0~1
        pcl_cloud.points.push_back(point);
    }

    // 生成pcd文件名
    std::string pcd_filename = "/home/dearmoon/datasets/NWU/日晴不颠簸高速4/lidar_pcd/lidar_pcd_" + std::to_string(lidar_frame_count) + ".pcd";
    // 保存PointCloud到pcd文件
    pcl::io::savePCDFileASCII(pcd_filename, pcl_cloud);
    
    ROS_INFO_STREAM("Saved " << cloud_msg.point_num << " points to " << pcd_filename);

    // 增加帧序号
    lidar_frame_count++;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_to_pcd");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/ars548_process/detection_point_cloud", 10, radarcloud_Callback);
    ros::Subscriber sub2 = nh.subscribe("/livox/lidar", 10, lidarcloud_Callback);


    ros::spin();

    return 0;
}