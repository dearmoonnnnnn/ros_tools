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


// 参数设置
const float max_distance = 1.0;  // 邻域搜索的最大距离
const float max_intensity_diff = 10.0;  // 强度差阈值

class RegionGrowingNode {
public:
    RegionGrowingNode() : nh_("~"){
        // 初始化订阅和发布
        sub_millimeter_cloud_.subscribe(nh_, "/ars548_process/detectioin_PointCloud2", 1);
        sub_lidar_cloud_.subscribe(nh_, "/livox/lidar_PointCloud2", 1);
        
        sync_.reset(new Sync(SyncPolicy(10), sub_millimeter_cloud_, sub_lidar_cloud_));
        sync_->registerCallback(boost::bind(&RegionGrowingNode::processClouds, this, _1, _2));

        pub_result_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/region_growing_cloud", 1);

        std::string outputFile = nh_.param<std::string>("outputFile","/test.bag");
        // std::cout << "outputFile: " << outputFile << std::endl;

        bag_.open(outputFile, rosbag::bagmode::Write);
    }

    ~RegionGrowingNode(){
        bag_.close();
    }

private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_millimeter_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_lidar_cloud_;
    ros::Publisher pub_result_cloud_;
    rosbag::Bag bag_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    void processClouds(const sensor_msgs::PointCloud2::ConstPtr& millimeter_msg,
                       const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
        // 转换点云格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr millimeter_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*millimeter_msg, *millimeter_cloud);
        pcl::fromROSMsg(*lidar_msg, *lidar_cloud);

        // 输出点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // 构建激光雷达点云 KD-Tree
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(lidar_cloud);

        // 遍历毫米波点云，找到种子点并执行区域生长
        for (const auto& seed_point : millimeter_cloud->points) {
            std::queue<int> seed_queue;
            std::vector<bool> processed(lidar_cloud->size(), false);

            // 查找种子点邻域
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_distances;

            kdtree.radiusSearch(seed_point, max_distance, neighbor_indices, neighbor_distances);

            // 将符合条件的点加入种子队列
            for (int idx : neighbor_indices) {
                if (!processed[idx] && canGrow(seed_point, lidar_cloud->points[idx])) {
                    seed_queue.push(idx);
                }
            }

            // 区域生长
            while (!seed_queue.empty()) {
                int idx = seed_queue.front();
                seed_queue.pop();

                if (processed[idx]) continue;

                processed[idx] = true;
                result_cloud->points.push_back(lidar_cloud->points[idx]);

                // 查找邻域点
                kdtree.radiusSearch(lidar_cloud->points[idx], max_distance, neighbor_indices, neighbor_distances);

                for (int neighbor_idx : neighbor_indices) {
                    if (!processed[neighbor_idx] && canGrow(lidar_cloud->points[idx], lidar_cloud->points[neighbor_idx])) {
                        seed_queue.push(neighbor_idx);
                    }
                }
            }
        }

        // 发布结果点云
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*result_cloud, output);
        output.header.frame_id = "map";
        output.header.stamp = lidar_msg->header.stamp;

        bag_.write("/RegionGrowing", output.header.stamp, output);
        // pub_result_cloud_.publish(output);

        ROS_INFO("Region growing completed!");
    }

    bool canGrow(const pcl::PointXYZI& seed, const pcl::PointXYZI& neighbor) {
        float distance = std::sqrt(std::pow(seed.x - neighbor.x, 2) +
                                   std::pow(seed.y - neighbor.y, 2) +
                                   std::pow(seed.z - neighbor.z, 2));
        return (distance < max_distance && std::abs(seed.intensity - neighbor.intensity) < max_intensity_diff);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "region_growing_node");
    RegionGrowingNode node;
    ros::spin();
    return 0;
}
