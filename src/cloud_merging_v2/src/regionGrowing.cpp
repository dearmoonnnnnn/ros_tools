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

cv::Mat Radar_to_Livox = (cv::Mat_<double>(4, 4) <<
 0.994838, 0.0187061, -0.0997379, -0.0379673, 
 -0.0209378, 0.999552, -0.0213763, -0.120289,
 0.0992934, 0.0233543, 0.994784, 0.41831,
 0,  0,  0,  1);

cv::Mat livox_to_Radar = (cv::Mat_<double>(4, 4) <<
 0.994838,  -0.0209378,  0.0992934, -0.0062827,
 0.0187061,  0.999552,   0.0233543,  0.11117599,
 -0.0997379, -0.0213763, 0.994784, -0.42248621,
  0,       0,        0,         1);       


// 参数设置
const float radar_to_lidar_distance = 1.0;  // 激光种子点与毫米波点的最大距离 
const float max_distance = 1.0;             // 邻域搜索的最大距离
const float max_intensity_diff = 10.0;      // 强度差阈值

class RegionGrowingNode {
public:
    RegionGrowingNode() : nh_("~"){
        start_time = ros::Time::now();
        // 初始化订阅和发布
        sub_millimeter_cloud_.subscribe(nh_, "/ars548_process/detectioin_PointCloud2", 4000);
        sub_lidar_cloud_.subscribe(nh_, "/livox/lidar_PointCloud2", 4000);
        
        sync_.reset(new Sync(SyncPolicy(4000), sub_millimeter_cloud_, sub_lidar_cloud_));
        sync_->registerCallback(boost::bind(&RegionGrowingNode::processClouds, this, _1, _2));

        pub_result_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/region_growing_cloud", 1);

        std::string outputFile = nh_.param<std::string>("outputFile","/test.bag");
        // std::cout << "outputFile: " << outputFile << std::endl;

        bag_.open(outputFile, rosbag::bagmode::Write);

    }

    ~RegionGrowingNode(){
        bag_.close();
        ROS_INFO("Total msg number is %d", msg_count);
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
    int msg_count = 0;
    ros::Duration total_time_;  // 累积的总处理时间
    ros::Time start_time;
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

        std::vector<bool> processed(lidar_cloud->size(), false);

        // 遍历毫米波点云，找到种子点并执行区域生长
        for (const auto& radar_point : millimeter_cloud->points) {
            // 将毫米波点云点转换到激光雷达坐标系
            pcl::PointXYZI seed_point;
            transformRadarToLidar(radar_point, seed_point);
            
            std::queue<int> seed_queue;

            // 查找种子点邻域
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_distances;

            if (kdtree.radiusSearch(seed_point, radar_to_lidar_distance, neighbor_indices, neighbor_distances) == 0){
                // ROS_INFO("No lidar seeds found");
                continue;
            }

            // 将符合条件的点加入种子队列
            for (int idx : neighbor_indices) {
                if (!processed[idx] && canGrow(seed_point, lidar_cloud->points[idx], true)) {
                    seed_queue.push(idx);
                }
                break;  // 只要最近的第一个雷达点
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

        // ROS_INFO("Region growing completed!");


        // 计算并累计处理时间
        total_time_ = ros::Time::now() - start_time;
        
        ROS_INFO("Processed %d msgs, total processing time: %f seconds", msg_count++, total_time_.toSec());        
        
    }

    bool canGrow(const pcl::PointXYZI& seed, const pcl::PointXYZI& neighbor, bool is_initial_search = false) {
        float distance = std::sqrt(std::pow(seed.x - neighbor.x, 2) +
                                std::pow(seed.y - neighbor.y, 2) +
                                std::pow(seed.z - neighbor.z, 2));
        if (is_initial_search) {
            return distance < max_distance;  // 初次搜索只考虑距离
        } else {
            return (distance < max_distance && 
                    std::abs(seed.intensity - neighbor.intensity) < max_intensity_diff);  // 后续搜索考虑强度
        }
    }


    void transformRadarToLidar(const pcl::PointXYZI& radar_point, pcl::PointXYZI& lidar_point) {
        // 定义变换矩阵
        static cv::Mat radar_to_lidar = Radar_to_Livox;

        // 构建齐次坐标点
        cv::Mat radar_point_mat = (cv::Mat_<double>(4, 1) <<
            radar_point.x, radar_point.y, radar_point.z, 1.0);

        // 执行变换
        cv::Mat lidar_point_mat = radar_to_lidar * radar_point_mat;

        // 提取结果
        lidar_point.x = lidar_point_mat.at<double>(0, 0);
        lidar_point.y = lidar_point_mat.at<double>(1, 0);
        lidar_point.z = lidar_point_mat.at<double>(2, 0);
        lidar_point.intensity = radar_point.intensity;  // 保留强度信息
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "region_growing_node");
    RegionGrowingNode node;
    ros::spin();
    return 0;
}
