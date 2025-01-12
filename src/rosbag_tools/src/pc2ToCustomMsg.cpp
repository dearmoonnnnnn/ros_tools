#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <livox_ros_driver/CustomMsg.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>  // 包含 pcl_ros 中的转换函数

// 将 PointCloud2 转换为 CustomMsg
void convertPointCloud2ToCustomMsg(const sensor_msgs::PointCloud2 &input, livox_ros_driver::CustomMsg &output, uint8_t lidar_id) {
    // 初始化 Header
    output.header = input.header;

    // Timebase: 将 header.stamp 转换为 uint64_t（单位：纳秒）
    output.timebase = static_cast<uint64_t>(input.header.stamp.sec) * 1e9 + input.header.stamp.nsec;

    // 设置 Lidar ID
    output.lidar_id = lidar_id;

    // 初始化保留字段
    output.rsvd = {0, 0, 0};

    // 将 PointCloud2 转换为 PCL 点云格式
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(input, pcl_cloud);

    // 点云数量
    output.point_num = pcl_cloud.size();
    output.points.clear();

    // 遍历 PCL 点云
    for (const auto &point : pcl_cloud.points) {
        livox_ros_driver::CustomPoint custom_point;

        // 坐标数据
        custom_point.x = point.x;
        custom_point.y = point.y;
        custom_point.z = point.z;

        // 反射率（假设 pcl::PointXYZI 中的 I 字段表示反射率）
        custom_point.reflectivity = static_cast<uint8_t>(point.intensity);

        // 偏移时间（假设所有点共享相同的时间戳）
        custom_point.offset_time = 0;

        // 添加点到 CustomMsg
        output.points.push_back(custom_point);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_to_custommsg_bag_writer");
    ros::NodeHandle nh("~");

    // 初始化 Bag 文件
    std::string bag_file;
    if (!nh.getParam("outputFile", bag_file)) {
        ROS_ERROR("Failed to get 'outputFile' parameter");
        return -1;
    } 

    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Write);

    ros::Duration total_time;
    int msg_count = 0;    

    ros::Time start_time = ros::Time::now();

    // 订阅 PointCloud2 消息
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/RegionGrowing", 4000,
        [&](const sensor_msgs::PointCloud2::ConstPtr &msg) {
            // 转换为 CustomMsg
            livox_ros_driver::CustomMsg custom_msg;
            uint8_t lidar_id = 1; // 示例 Lidar ID
            convertPointCloud2ToCustomMsg(*msg, custom_msg, lidar_id);

            // 写入 Bag 文件，时间戳使用输入消息的时间戳
            bag.write("/livox/lidar", msg->header.stamp, custom_msg);

            ros::Duration total_time = ros::Time::now() - start_time;

            ROS_INFO("Written the %d CustomMsg with %u points to bag, total propressing time: %f seconds", 
                     msg_count++,
                     custom_msg.point_num,
                     total_time.toSec());
        });

    ros::Rate rate(10);  // 设置循环频率
    while (ros::ok()) {
        ros::spinOnce();  // 处理回调队列
        rate.sleep();
    }

    // 关闭 Bag 文件
    bag.close();    

    return 0;
}
