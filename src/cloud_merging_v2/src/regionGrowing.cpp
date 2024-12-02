#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/projections.h>
#include <queue>
#include <cmath>


/* 
 * 输入：PointCloud2 格式的激光雷达和毫米波雷达点云
 * 生长条件：判断种子点与邻域点之间的距离和强度差是否符合阈值。
 * 输出：区域生长后的点云
 */

// 点类型定义
struct PointXYZIntensity {
    PCL_ADD_POINT4D;
    float intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIntensity,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity))

// 参数设置
const float max_distance = 1.0;  // 邻域搜索的最大距离
const float max_intensity_diff = 10.0;  // 强度差阈值

class RegionGrowingNode {
public:
    RegionGrowingNode() {
        // 初始化订阅和发布
        sub_millimeter_cloud_ = nh_.subscribe("/millimeter_cloud", 1, &RegionGrowingNode::millimeterCallback, this);
        sub_lidar_cloud_ = nh_.subscribe("/lidar_cloud", 1, &RegionGrowingNode::lidarCallback, this);
        pub_result_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/region_growing_cloud", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_millimeter_cloud_;
    ros::Subscriber sub_lidar_cloud_;
    ros::Publisher pub_result_cloud_;

    pcl::PointCloud<PointXYZIntensity>::Ptr millimeter_cloud_{new pcl::PointCloud<PointXYZIntensity>()};
    pcl::PointCloud<PointXYZIntensity>::Ptr lidar_cloud_{new pcl::PointCloud<PointXYZIntensity>()};
    bool millimeter_ready_ = false;
    bool lidar_ready_ = false;

    void millimeterCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::fromROSMsg(*msg, *millimeter_cloud_);
        millimeter_ready_ = true;
        processClouds();
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::fromROSMsg(*msg, *lidar_cloud_);
        lidar_ready_ = true;
        processClouds();
    }

    bool canGrow(const PointXYZIntensity& seed, const PointXYZIntensity& neighbor) {
        float distance = std::sqrt(std::pow(seed.x - neighbor.x, 2) +
                                   std::pow(seed.y - neighbor.y, 2) +
                                   std::pow(seed.z - neighbor.z, 2));
        return (distance < max_distance && std::abs(seed.intensity - neighbor.intensity) < max_intensity_diff);
    }

    void processClouds() {
        if (!millimeter_ready_ || !lidar_ready_) return;

        // 输出点云
        pcl::PointCloud<PointXYZIntensity>::Ptr result_cloud(new pcl::PointCloud<PointXYZIntensity>());

        // 构建激光雷达点云 KD-Tree
        pcl::KdTreeFLANN<PointXYZIntensity> kdtree;
        kdtree.setInputCloud(lidar_cloud_);

        // 遍历毫米波点云，找到种子点并执行区域生长
        for (const auto& seed_point : millimeter_cloud_->points) {
            std::queue<int> seed_queue;
            std::vector<bool> processed(lidar_cloud_->size(), false);

            // 查找种子点邻域
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_distances;

            kdtree.radiusSearch(seed_point, max_distance, neighbor_indices, neighbor_distances);

            // 将符合条件的点加入种子队列
            for (int idx : neighbor_indices) {
                if (!processed[idx] && canGrow(seed_point, lidar_cloud_->points[idx])) {
                    seed_queue.push(idx);
                }
            }

            // 区域生长
            while (!seed_queue.empty()) {
                int idx = seed_queue.front();
                seed_queue.pop();

                if (processed[idx]) continue;

                processed[idx] = true;
                result_cloud->points.push_back(lidar_cloud_->points[idx]);

                // 查找邻域点
                kdtree.radiusSearch(lidar_cloud_->points[idx], max_distance, neighbor_indices, neighbor_distances);

                for (int neighbor_idx : neighbor_indices) {
                    if (!processed[neighbor_idx] && canGrow(lidar_cloud_->points[idx], lidar_cloud_->points[neighbor_idx])) {
                        seed_queue.push(neighbor_idx);
                    }
                }
            }
        }

        // 发布结果点云
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*result_cloud, output);
        output.header.frame_id = "map";
        pub_result_cloud_.publish(output);

        ROS_INFO("Region growing completed!");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "region_growing_node");
    RegionGrowingNode node;
    ros::spin();
    return 0;
}
