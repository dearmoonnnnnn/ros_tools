#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg1, const sensor_msgs::PointCloud2::ConstPtr& msg2) {
    ROS_INFO("msg1 frame_id: %s", msg1->header.frame_id.c_str());
    ROS_INFO("msg1 timestamp: %f", msg1->header.stamp.toSec());

    ROS_INFO("msg2 frame_id: %s", msg2->header.frame_id.c_str());
    ROS_INFO("msg2 timestamp: %f", msg2->header.stamp.toSec());
    
    ROS_INFO("---------------------------");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sync_sub_1");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, "/topic1", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh, "/topic2", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);

    sync.registerCallback(boost::bind(&callback, _1, _2));
    // sync.setMaxIntervalDuration(ros::Duration(1000000000));


    ros::spin();

    return 0;
}
