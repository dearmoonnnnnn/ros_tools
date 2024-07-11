#inlcude <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs::PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <cloud_merging/include/point_xyzidv.h>

// 参数初始化
void initializeParams() {

    cloud_callback_count = 0;
    imu_callback_count = 0;

    // 降采样方法、分辨率
    std::string downsample_method = private_nh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);

    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      auto voxelgrid = new pcl::VoxelGrid<PointT>();                                                  // 创建pcl::VoxelGrid对象，用于进行体素网格降采样
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);    // 设置体素网格的叶子大小，即降采样的分辨率
      downsample_filter.reset(voxelgrid);                                                             // 将 downsample_filter 重置为新创建的 pcl::VoxelGrid 对象
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>()); // 用于进行近似体素网格降采样
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;                                                             // 将downsample_filer赋值为approx_voxelgrid
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
    }

    // 去除离群点的方法，可选值为"STATISTICAL"、"RADIUS",即"统计方法"、"半径方法"
    std::string outlier_removal_method = private_nh.param<std::string>("outlier_removal_method", "STATISTICAL");
    if(outlier_removal_method == "STATISTICAL") {
      int mean_k = private_nh.param<int>("statistical_mean_k", 20);                   // 计算邻域均值的点数
      double stddev_mul_thresh = private_nh.param<double>("statistical_stddev", 1.0); // 标准差的倍数阈值
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;
      
      //创建统计离群点移除对象
      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;   // 将智能指针指向统计离群点移除对象
    } else if(outlier_removal_method == "RADIUS") {
      double radius = private_nh.param<double>("radius_radius", 0.8);                   // 半径
      int min_neighbors = private_nh.param<int>("radius_min_neighbors", 2);             // 领域最小点数
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;
      
      // 创建半径离群点移除对象
      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter = rad;   // 将智能指针指向半径离群点移除对象
    } 
    // else if (outlier_removal_method == "BILATERAL")
    // {
    //   double sigma_s = private_nh.param<double>("bilateral_sigma_s", 5.0);
    //   double sigma_r = private_nh.param<double>("bilateral_sigma_r", 0.03);
    //   std::cout << "outlier_removal: BILATERAL " << sigma_s << " - " << sigma_r << std::endl;

    //   pcl::FastBilateralFilter<PointT>::Ptr fbf(new pcl::FastBilateralFilter<PointT>());
    //   fbf->setSigmaS (sigma_s);
    //   fbf->setSigmaR (sigma_r);
    //   outlier_removal_filter = fbf;
    // }
     else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }

    // 距离过滤相关参数
    use_distance_filter = private_nh.param<bool>("use_distance_filter", true);    // 表示是否使用距离过滤
    distance_near_thresh = private_nh.param<double>("distance_near_thresh", 1.0); // 距离过滤的近的阈值
    distance_far_thresh = private_nh.param<double>("distance_far_thresh", 100.0); // 距离过滤的远的阈值
    // 点云在z轴上的高度范围
    z_low_thresh = private_nh.param<double>("z_low_thresh", -5.0);                
    z_high_thresh = private_nh.param<double>("z_high_thresh", 20.0);

    // 从参数服务器获取ground truth文件路径和是否进行tf发布的参数
    std::string file_name = private_nh.param<std::string>("gt_file_location", "");
    publish_tf = private_nh.param<bool>("publish_tf", false);

    ifstream file_in(file_name);
    if (!file_in.is_open()) {
        cout << "Can not open this gt file" << endl;
    }
    else{
      //将文件中的每一行存储到vector中
      std::vector<std::string> vectorLines;
      std::string line;
      while (getline(file_in, line)) {
          vectorLines.push_back(line);
      }
      
      for (size_t i = 1; i < vectorLines.size(); i++) {
          std::string line_ = vectorLines.at(i);                    // 获取vector中的每一行
          double stamp,tx,ty,tz,qx,qy,qz,qw;                        // 定义相关变量
          stringstream data(line_);                                 // 使用字符串流读取当前行的数据
          data >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;  // 解析当前行的数据
          nav_msgs::Odometry odom_msg;                              // 创建Odometry消息对象
          // odom_msg的头部信息
          odom_msg.header.frame_id = mapFrame;                      // 该消息所在的坐标系的名称            
          odom_msg.child_frame_id = baselinkFrame;                  // 表示一个相对于frame_id的子坐标系。如激光雷达数据的child_frame_id可能是base_laser,表示激光雷达相对于机器人底盘坐标系的位置。
          odom_msg.header.stamp = ros::Time().fromSec(stamp);
          // odom_msg的姿态信息
          odom_msg.pose.pose.orientation.w = qw;
          odom_msg.pose.pose.orientation.x = qx;
          odom_msg.pose.pose.orientation.y = qy;
          odom_msg.pose.pose.orientation.z = qz;
          // odom_msg的位置信息
          odom_msg.pose.pose.position.x = tx;
          odom_msg.pose.pose.position.y = ty;
          odom_msg.pose.pose.position.z = tz;
          std::lock_guard<std::mutex> lock(odom_queue_mutex);
          odom_msgs.push_back(odom_msg);
      }
    }
    file_in.close();
}


// 点云的区域截取，输入点云中 z 坐标在 -2 和 10 之间的点截取出来，返回一个新的点云
pcl::PointCloud<PointT>::ConstPtr passthrough(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    PointT pt;
    for(int i = 0; i < cloud->size(); i++){
      if (cloud->at(i).z < 10 && cloud->at(i).z > -2){
        pt.x = (*cloud)[i].x;
        pt.y = (*cloud)[i].y;
        pt.z = (*cloud)[i].z;
        pt.intensity = (*cloud)[i].intensity;
        filtered->points.push_back(pt);
      }
    }
    filtered->header = cloud->header;
    return filtered;
}

// 下采样
pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {      // 若不存在下采样滤波器，移除NaN/Inf点并返回
      // Remove NaN/Inf points
      pcl::PointCloud<PointT>::Ptr cloudout(new pcl::PointCloud<PointT>());
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloudout, indices);
      
      return cloudout;
    }

    // 若存在下采样滤波器，创建一个新的点云对象用于存储下采样后的点云
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
}

// 离群点移除函数
pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
}


// 距离过滤函数
pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

    filtered->reserve(cloud->size());                     // 为过滤后的点云预先分配空间


    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
      double d = p.getVector3fMap().norm();               // 计算点到原点的距离
      double z = p.z;                                     // 获取点的z坐标
      // 根据设定的距离阈值和Z坐标范围过滤点云
      return d > distance_near_thresh && d < distance_far_thresh && z < z_high_thresh && z > z_low_thresh;
    });
    // for (size_t i=0; i<cloud->size(); i++){
    //   const PointT p = cloud->points.at(i);
    //   double d = p.getVector3fMap().norm();
    //   double z = p.z;
    //   if (d > distance_near_thresh && d < distance_far_thresh && z < z_high_thresh && z > z_low_thresh)
    //     filtered->points.push_back(p);
    // }

    // 设置过滤后的点云的宽度、高度和稠密属性
    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    // 将过滤后的点云的头信息设置为与输入点云相同
    filtered->header = cloud->header;

    return filtered;
}


// 去扭曲
pcl::PointCloud<PointT>::ConstPtr deskewing(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    ros::Time stamp = pcl_conversions::fromPCL(cloud->header.stamp);
    if(imu_queue.empty()) {           // 如果IMU队列为空，直接返回原始点云(无法进行去畸变)
      return cloud;
    }

    // the color encodes the point number in the point sequence
    if(colored_pub.getNumSubscribers()) {     // 如果有节点订阅了带颜色的点云，将带有颜色的点云发布出去
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
      colored->header = cloud->header;
      colored->is_dense = cloud->is_dense;
      colored->width = cloud->width;
      colored->height = cloud->height;
      colored->resize(cloud->size());

      for(int i = 0; i < cloud->size(); i++) {
        double t = static_cast<double>(i) / cloud->size();
        colored->at(i).getVector4fMap() = cloud->at(i).getVector4fMap();
        colored->at(i).r = 255 * t;
        colored->at(i).g = 128;
        colored->at(i).b = 255 * (1 - t);
      }
      colored_pub.publish(*colored);
    }

    // 从IMU队列中获取最早的IMU数据，用于去畸变
    sensor_msgs::ImuConstPtr imu_msg = imu_queue.front();

    // 寻找IMU队列中最早的时间戳，以与当前点云对齐
    auto loc = imu_queue.begin();
    for(; loc != imu_queue.end(); loc++) {
      imu_msg = (*loc);
      if((*loc)->header.stamp > stamp) {
        break;
      }
    }

    // 从IMU队列中移除已使用的IMU数据
    imu_queue.erase(imu_queue.begin(), loc);

    // 获取IMU的角度速度，并反转
    Eigen::Vector3f ang_v(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    ang_v *= -1;

    // deskewed存储去畸变后的点云
    pcl::PointCloud<PointT>::Ptr deskewed(new pcl::PointCloud<PointT>());
    deskewed->header = cloud->header;
    deskewed->is_dense = cloud->is_dense;
    deskewed->width = cloud->width;
    deskewed->height = cloud->height;
    deskewed->resize(cloud->size());

    // 获取扫描周期
    double scan_period = private_nh.param<double>("scan_period", 0.1);
    // 遍历每个点，进行去畸变操作
    for(int i = 0; i < cloud->size(); i++) {
      const auto& pt = cloud->at(i);

      // TODO: transform IMU data into the LIDAR frame
      double delta_t = scan_period * static_cast<double>(i) / cloud->size();          // 点 i 在扫描周期 scan_period 内的时间偏移
      Eigen::Quaternionf delta_q(1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2]);  // 在 delta_t 内发生的旋转
      Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();                  // 将点的三维坐标乘以四元数的逆来应用去畸变变换，并转换成 Vector3f 类型

      // 将去畸变后的点添加到新的点云中
      deskewed->at(i) = cloud->at(i);
      deskewed->at(i).getVector3fMap() = pt_;
    }

    return deskewed;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr&  msg) { 
    pcl::PointXYZIDV radarpoint_raw;            // 点，带有x、y、z、强度和多普勒速度信息
    pcl::PointCloud<pcl::PointXYZIDV>::Ptr radarcloud_raw(new pcl::PointCloud<pcl::PointXYZIDV>());

    // 遍历每一个点
    pcl::PointCloud<pcl::PointXYZIDV> radarcloud_xyzidv;
    pcl::fromROSMsg(*msg, radarcloud_xyzidv);

    for(const auto& point : radarcloud_xyzidv)
    {

        if( point.intensity > power_threshold) //"Power"
        {
            // 检查点的坐标是否无效(NaN或无穷大)
            if (point.x == NAN || point.y == NAN || point.z == NAN) continue;
            if (point.x == INFINITY || point.y == INFINITY || point.z == INFINITY) continue;


            // 对点进行赋值
            radarpoint_raw.x = point.x;
            radarpoint_raw.y = point.y;
            radarpoint_raw.z = point.z;
            radarpoint_raw.intensity = point.intensity;
            radarpoint_raw.doppler = point.doppler_velocity;

            // 将点添加到点云中
            radarcloud_raw->points.push_back(radarpoint_raw);

        }
        
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "precessing");
    ros::NodeHandle nh;

    pointCloudTopic = nh.param<std::string>("point_cloud_topic", "/ars548_process/detection_point_cloud");
    points_sub = nh.subscribe(pointCloudTopic, 64, &PreprocessingNodelet::cloud_callback, this);



}