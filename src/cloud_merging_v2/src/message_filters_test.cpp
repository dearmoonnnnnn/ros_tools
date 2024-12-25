#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <queue>
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rosbag/bag.h>
#include <opencv2/opencv.hpp>


/*
 * 测试时间同步
 */


class RegionGrowingNode {
public:
    RegionGrowingNode() : nh_("~"){
        // 初始化订阅和发布
        sub_millimeter_cloud_.subscribe(nh_, "/ars548_process/detectioin_PointCloud2", 1);
        sub_lidar_cloud_.subscribe(nh_, "/livox/lidar_PointCloud2", 1);
        
        sync_.reset(new Sync(SyncPolicy(512), sub_millimeter_cloud_, sub_lidar_cloud_));
        sync_->registerCallback(boost::bind(&RegionGrowingNode::processClouds, this, _1, _2));

        pub_result_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/region_growing_cloud", 1);

    }

    ~RegionGrowingNode(){
        ROS_INFO("Total msg number is %d", msg_count);
    }

private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_millimeter_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_lidar_cloud_;
    ros::Publisher pub_result_cloud_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
    int msg_count = 0;

    void processClouds(const sensor_msgs::PointCloud2::ConstPtr& millimeter_msg,
                       const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
        ROS_INFO("Current msg number is %d", msg_count++);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "region_growing_node");
    RegionGrowingNode node;
    ros::spin();
    return 0;
}
