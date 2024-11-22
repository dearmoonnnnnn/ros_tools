#include <rosbag/bag.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <unordered_map>
#include <iomanip>

bool isNameValid(const std::string& name){
    if (name == "IMU.FDI_ROLL" || name == "IMU.FDI_PITCH" || name == "IMU.FDI_YAW"){
        return true;
    }

    if (name == "IMU.IMU_RATEX" || name == "IMU.IMU_RATEY" || name == "IMU.IMU_RATEZ"){
        return true;
    }

    if (name == "IMU.IMU_ACCX" || name == "IMU.IMU_ACCY" || name == "IMU.IMU_ACCZ"){
        return true;
    }

    return false;

}

// 欧拉角转四元数
void eulerToQuaternion(double roll, double pitch, double yaw, double &qw, double &qx, double &qy, double &qz) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_txt_to_bag");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    std::string input_file, output_file;

    if (!nh.getParam("/input_file", input_file)) {
        ROS_ERROR("Failed to get 'input_file' parameter");
        return -1;
    }

    if (!nh.getParam("/output_file", output_file)) {
        ROS_ERROR("Failed to get 'output_file' parameter");
        return -1;
    }


    std::ifstream file(input_file); // 打开数据文件
    if (!file.is_open()) {
        ROS_ERROR("Failed to open the file!");
        return -1;
    }


    bag.open(output_file, rosbag::bagmode::Write);

    std::unordered_map<std::string, double> data; // 存储字段与值的对应关系
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double timestamp;
        int id; // 可忽略
        std::string name;
        double value;

        // 解析一行数据
        if (!(iss >> timestamp >> id >> name >> value)) {
            ROS_WARN("Malformed line: %s", line.c_str());
            continue;
        }

        if (!isNameValid(name)){
            ROS_WARN("Invalid name : %s", name.c_str());
            continue;
        }

        if (1){
            std::cout << "name : " << name << std::endl;
            std::cout << "value : " << std::fixed << std::setprecision(20) << value << std::endl;
        }
        
        data[name] = value; // 保存字段值

        // 当所有需要的字段被读取时，生成并写入 bag
        if (data.size() >= 9) { // 确保有足够数据
            sensor_msgs::Imu imu_msg;

            // 时间戳
            imu_msg.header.stamp.sec = static_cast<int>(timestamp);
            imu_msg.header.stamp.nsec = static_cast<int>((timestamp - imu_msg.header.stamp.sec) * 1e9);
            imu_msg.header.frame_id = "imu_link";

            // 欧拉角转换为四元数
            double roll = data["IMU.FDI_ROLL"] * M_PI / 180.0;
            double pitch = data["IMU.FDI_PITCH"] * M_PI / 180.0;
            double yaw = data["IMU.FDI_YAW"] * M_PI / 180.0;
            double qw, qx, qy, qz;
            eulerToQuaternion(roll, pitch, yaw, qw, qx, qy, qz);

            if(1){
                std::cout << "data[\"IMU.FDI_ROLL\"] : " << data["IMU.FDI_ROLL"] << std::endl;
                std::cout << "data[\"IMU.FDI_PITCH\"] : " << data["IMU.FDI_PITCH"] << std::endl;
                std::cout << "data[\"IMU.FDI_YAW\"] : " << data["IMU.FDI_YAW"] << std::endl;
                std::cout << "roll angle in radians : " << roll << std::endl;
                std::cout << "pitch angle in radians : " << pitch << std::endl;
                std::cout << "yaw angle in radians : " << yaw << std::endl;
                std::cout << "qw : " << qw << std::endl;
                std::cout << "qx : " << qx << std::endl;
                std::cout << "qy : " << qy << std::endl;
                std::cout << "qz : " << qz << std::endl;
                std::cout << "--------------- one msg done ----------------" << std::endl;
            }

            imu_msg.orientation.w = qw;
            imu_msg.orientation.x = qx;
            imu_msg.orientation.y = qy;
            imu_msg.orientation.z = qz;

            // 角速度
            imu_msg.angular_velocity.x = data["IMU.IMU_RATEX"];
            imu_msg.angular_velocity.y = data["IMU.IMU_RATEY"];
            imu_msg.angular_velocity.z = data["IMU.IMU_RATEZ"];

            // 线性加速度
            imu_msg.linear_acceleration.x = data["IMU.IMU_ACCX"];
            imu_msg.linear_acceleration.y = data["IMU.IMU_ACCY"];
            imu_msg.linear_acceleration.z = data["IMU.IMU_ACCZ"];

            // 写入 bag 文件
            bag.write("/vectornav/imu", imu_msg.header.stamp, imu_msg);

            data.clear(); // 清空数据，准备读取下一组
        }
    }

    file.close();
    bag.close(); // 关闭 bag 文件
    ROS_INFO("IMU data has been saved to %s", output_file.c_str());

    return 0;
}
