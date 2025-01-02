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
int msg_count;

void callback(const livox_ros_driver::CustomMsg::ConstPtr& region_growing_cloud_, const livox_ros_driver::CustomMsg::ConstPtr& origin_cloud_){
    ROS_INFO("msg number : %d", msg_count++);
    // 创建一个新的 CustomMsg 用来保存填充后的点云数据
    livox_ros_driver::CustomMsg result_cloud;

    // 填充全局信息字段
    result_cloud.header = region_growing_cloud_->header;  // 直接保留 header
    result_cloud.timebase = origin_cloud_->timebase;      // 填充 timebase
    result_cloud.lidar_id = origin_cloud_->lidar_id;      // 填充 lidar_id
    result_cloud.rsvd = origin_cloud_->rsvd;              // 填充 reserved 字段
    result_cloud.point_num = region_growing_cloud_->points.size();  // 设置点云的数量

    // 遍历 region_growing_cloud_ 中的点
    for (size_t i = 0; i < region_growing_cloud_->points.size(); ++i) {
        const auto& region_point = region_growing_cloud_->points[i];

        // 查找 origin_cloud_ 中对应的点
        bool found = false;
        for (size_t j = 0; j < origin_cloud_->points.size(); ++j) {
            const auto& origin_point = origin_cloud_->points[j];

            // 比较坐标和 reflectivity（反射强度），认为相同的点就是对应的
            if (std::abs(region_point.x - origin_point.x) < 1e-6 &&
                std::abs(region_point.y - origin_point.y) < 1e-6 &&
                std::abs(region_point.z - origin_point.z) < 1e-6 &&
                std::abs(region_point.reflectivity - origin_point.reflectivity) < 1e-6) {

                // 如果找到了对应的点，填充其他字段
                livox_ros_driver::CustomPoint filled_point = region_point;

                // 填充其他字段
                filled_point.offset_time = origin_point.offset_time;
                filled_point.tag = origin_point.tag;
                filled_point.line = origin_point.line;

                // 将填充后的点加入 result_cloud 中
                result_cloud.points.push_back(filled_point);
                found = true;
                break; // 找到后就可以跳出当前的内层循环
            }
        }

        // 如果没有找到对应的点，则可以选择丢弃该点或做其他处理
        if (!found) {
            ROS_WARN("No corresponding point found in origin cloud for point %zu", i);
        }
    }
    

    // 写入到 rosbag
    bag.write("/FilledRegionGrowingCloud", result_cloud.header.stamp, result_cloud);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "CustomMsgFilling");
    ros::NodeHandle nh("~");

    if (!nh.getParam("outputPath", output_path_)){
        ROS_ERROR("get parameter 'outputPath' error");
        return -1;
    }

    bag.open(output_path_, rosbag::bagmode::Write);
    msg_count = 0;
    
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