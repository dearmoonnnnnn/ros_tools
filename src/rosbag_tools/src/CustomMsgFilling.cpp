#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <rosbag/bag.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
/*
 * 区域生长后的 CustomMsg 丢失了部分字段，需要重新填充
 * 将其与最原始的 CustomMsg 比对，若坐标点一样，则认为是同一个点，填充对应字段
 */

rosbag::Bag bag;
std::string output_path_;

void callback(const livox_ros_driver::CustomMsg::ConstPtr& region_growing_cloud_, const livox_ros_driver::CustomMsg::ConstPtr& origin_cloud_){


}

int main(int argc, char **argv){
    ros::init(argc, argv, "CustomMsgFilling");
    ros::NodeHandle nh("~");

    if (!nh.getParam("outputPath", output_path_)){
        ROS_ERROR("get parameter 'outputPath' error");
        return -1;
    }

    bag.open(output_path_, rosbag::bagmode::Write);
    
    // 订阅器
    message_filters::Subscriber<livox_ros_driver::CustomMsg> sub_region_growing_cloud_(nh, "/RegionGrowing/CustomMsg", 4000);
    message_filters::Subscriber<livox_ros_driver::CustomMsg> sub_origin_cloud_(nh, "/livox/lidar", 4000);

    // 同步策略
    typedef message_filters::sync_policies::ApproximateTime<
        livox_ros_driver::CustomMsg, livox_ros_driver::CustomMsg> SyncPolicy;

    // 创建并初始化同步器
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(4000), sub_region_growing_cloud_, sub_origin_cloud_);
    
    // 注册回调函数
    sync_->registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    bag.close();
    
    return 0;
}