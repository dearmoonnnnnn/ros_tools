#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

ros::Publisher pub;
static int frame_count = 0; // 添加静态变量记录帧序号

void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud_msg)
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
    std::string output_file = "/home/dearmoon/datasets/calib/NWU/pcd_2/" + std::to_string(frame_count) + ".pcd";

    pcl::io::savePCDFileASCII(output_file, cloud);
    ROS_INFO("Saved %lu data points to %s.", cloud.points.size(), output_file.c_str());

    frame_count++; // 增加帧序号
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_to_pcd");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/ars548_process/detection_point_cloud", 10, cloudCallback);

    ros::spin();

    return 0;
}