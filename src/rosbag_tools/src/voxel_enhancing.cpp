#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <rosbag/bag.h>

typedef pcl::PointCloud<pcl::PointXYZI> PCLPointCloud;
rosbag::Bag bag;

sensor_msgs::PointCloud enhancePointCloud(const sensor_msgs::PointCloud& inputCloud)
{
    // 将ROS的PointCloud转换为PCL的PointCloud
    PCLPointCloud::Ptr pclCloud(new PCLPointCloud);
    pclCloud->resize(inputCloud.points.size());

    for (size_t i = 0; i < inputCloud.points.size(); ++i)
    {
        const geometry_msgs::Point32& point = inputCloud.points[i];
        float doppler = inputCloud.channels[0].values[i];
        float intensity = inputCloud.channels[1].values[i];

        pcl::PointXYZI pclPoint;
        pclPoint.x = point.x;
        pclPoint.y = point.y;
        pclPoint.z = point.z;
        pclPoint.intensity = intensity;
        pclPoint.data[3] = doppler;

        pclCloud->points[i] = pclPoint;
    }

    // 设置体素化参数
    float voxelSize = 0.3f;  // 体素大小，根据实际情况调整

    // 对点云进行体素化下采样
    pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
    voxelGrid.setInputCloud(pclCloud);
    voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);
    PCLPointCloud::Ptr downsampledCloud(new PCLPointCloud);
    voxelGrid.filter(*downsampledCloud);

    // 对下采样后的点云进行体素化上采样
    voxelGrid.setInputCloud(downsampledCloud);
    voxelGrid.setLeafSize(voxelSize / 2, voxelSize / 2, voxelSize / 2);
    PCLPointCloud::Ptr upsampledCloud(new PCLPointCloud);
    voxelGrid.filter(*upsampledCloud);

    // 将PCL的PointCloud转换为ROS的PointCloud
    sensor_msgs::PointCloud enhancedCloud;
    enhancedCloud.header = inputCloud.header;
    enhancedCloud.points.resize(upsampledCloud->size());
    enhancedCloud.channels.resize(2);
    enhancedCloud.channels[0].name = "doppler";
    enhancedCloud.channels[0].values.resize(upsampledCloud->size());
    enhancedCloud.channels[1].name = "intensity";
    enhancedCloud.channels[1].values.resize(upsampledCloud->size());

    for (size_t i = 0; i < upsampledCloud->size(); ++i)
    {
        const pcl::PointXYZI& point = upsampledCloud->points[i];
        enhancedCloud.points[i].x = point.x;
        enhancedCloud.points[i].y = point.y;
        enhancedCloud.points[i].z = point.z;
        enhancedCloud.channels[0].values[i] = point.data[3];
        enhancedCloud.channels[1].values[i] = point.intensity;
    }

    return enhancedCloud;
}

void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
    // 对点云进行增强
    sensor_msgs::PointCloud enhancedCloud = enhancePointCloud(*cloud);

    // 将增强后的点云写入bag文件
    if (bag.isOpen()) {
        bag.write("/enhanced_point_cloud", cloud->header.stamp, enhancedCloud);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_enhancement");
    ros::NodeHandle nh;

    bag.open("/home/dearmoon/datasets/NWU/日晴不颠簸低速3/4DRadar/enhanced/enhanced_point_cloud.bag", rosbag::bagmode::Write);    
    // 订阅原始点云话题
    ros::Subscriber sub = nh.subscribe("/ars548_process/detection_point_cloud", 10, pointCloudCallback);

    ros::spin();

    bag.close();

    return 0;
}