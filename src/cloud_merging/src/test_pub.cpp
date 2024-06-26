#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "header_publisher_2");
    ros::NodeHandle nh;

    ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2>("/topic1", 10);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("/topic2", 10);


    sensor_msgs::PointCloud2 msg1;
    sensor_msgs::PointCloud2 msg2;

    msg1.header.frame_id = "A";
    ros::Time timeA;
    timeA.sec = 123456789;
    timeA.nsec = 1000156789;
    msg1.header.stamp = timeA;

    msg2.header.frame_id = "B";
    ros::Time timeB;
    timeB.sec = 123456789;
    timeB.nsec = 1000256789;
    msg2.header.stamp = timeB;

    ros::Rate rate(1);  // 1 Hz
    while (ros::ok()) {

        pub1.publish(msg1);

        pub2.publish(msg2);
        //pub2.publish(msg2);

        ROS_INFO("Published message to /topic1 with timestamp: %.9f", msg1.header.stamp.toSec());
        ROS_INFO("Published message to /topic2 with timestamp: %.9f", msg2.header.stamp.toSec());


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
