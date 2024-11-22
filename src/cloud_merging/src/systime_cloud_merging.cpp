#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <rosbag_tools/CustomMsg.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

/*
 *  step1:
 *  将时间同步后的两种数据写入一个bag包中
 */


// 从 ROS bag 文件中读取数据
// rosbag::Bag input_bag("/home/dearmoon/datasets/NWU/日晴不颠簸低速3/enhancing/radar_lidar_input.bag", rosbag::bagmode::Read);
rosbag::Bag output_bag;
int count = 0;

// 定义回调函数，处理同步后的消息
void callback(const sensor_msgs::PointCloud::ConstPtr& radar_msg, const rosbag_tools::CustomMsg::ConstPtr& lidar_msg)
{
    // 在这里处理同步后的消息
    ROS_INFO("radar Timestamp: %f", radar_msg->header.stamp.toSec());
    ROS_INFO("lidar Timestamp: %f", lidar_msg->header.stamp.toSec());
    ROS_INFO("---------------------------------------------------");
    
    if (output_bag.isOpen()){
        // 将处理后的消息写入新的 ROS bag 文件
        output_bag.write("/ars548_process/detection_point_cloud", radar_msg->header.stamp, *radar_msg);
        output_bag.write("/livox/lidar", lidar_msg->header.stamp, *lidar_msg);
        count++;
    }

}

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "time_sync_example_cpp");
    ros::NodeHandle nh;

    output_bag.open("/home/dearmoon/datasets/NWU/夜雪不颠簸高速/enhancing/test.bag", rosbag::bagmode::Write);
   
    // 定义 radar 和 PointCloud2 的订阅者
    message_filters::Subscriber<sensor_msgs::PointCloud> radar_sub(nh, "/ars548_process/detection_point_cloud", 10);
    message_filters::Subscriber<rosbag_tools::CustomMsg> lidar_sub(nh, "/livox/lidar", 10);

    // 定义时间同步策略，传入参数
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, rosbag_tools::CustomMsg> MySyncPolicy;
    
    // 定义同步器，传入同步策略和订阅对象
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(32), radar_sub, lidar_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    sync.setMaxIntervalDuration(ros::Duration(0.05));       // 设置最大时间间隔，单位秒

    // // 遍历 ROS bag 并发送消息
    // for (rosbag::MessageInstance const & msg : rosbag::View(input_bag))
    // {
    //     sensor_msgs::PointCloud::ConstPtr radar_msg = msg.instantiate<sensor_msgs::PointCloud>();
    //     rosbag_tools::CustomMsg::ConstPtr lidar_msg = msg.instantiate<rosbag_tools::CustomMsg>();
       
    //     if (msg.getTopic() == "/ars548_process/detection_point_cloud" && radar_msg != nullptr)
    //     {
    //         radar_sub->callback(*radar_msg);
    //     }
    //     else if (msg.getTopic() == "/livox/lidar" && lidar_msg != nullptr)
    //     {
    //         lidar_sub->callback(*lidar_msg);
    //     }
    // }

    

    // 保持节点活动
    ros::spin();

    // input_bag.close();
    output_bag.close();

    ROS_INFO("Finished processing bag file.");
    ROS_INFO("Total number of messages processed: %d", count);
    return 0;
}
