#include "utility.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl_ros/point_cloud.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <cmath>

class LidarMappingNode : public ParamServer
{
public:
    LidarMappingNode()
    {
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu", 10, &LidarMappingNode::imuCallback, this);
        lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &LidarMappingNode::lidarCallback, this);
        gps_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("ublox_gps/fix", 10, &LidarMappingNode::gpsCallback, this);
        ROS_INFO("\033[1;33m----> Loading for Map . . .\033[0m");
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(PathGlobalMap, global_map_) == -1)
        {
            ROS_ERROR("Couldn't read the global map PCD file");
            ros::shutdown();
            return;
        }
        ROS_INFO("\033[1;34mGlobal map loaded with %zu points ! ! \033[0m", global_map_.points.size());
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) 
    {
        if(gps_received_) return;
        _latitude = gps_msg->latitude;
        _longitude = gps_msg->longitude;
        _altitude = gps_msg->altitude;
        gps_received_ = true;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) 
    {
        if(imu_received_) return;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(imu_msg->orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        current_roll_ = roll;
        current_pitch_ = pitch;
        current_yaw_ = yaw;
        imu_received_ = true;
        ROS_INFO("IMU sub R P Y : %f %f %f",current_roll_, current_pitch_, current_yaw_);
    }


    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        // 첫 번째 포인트 클라우드 처리 후 노드 종료
        if (processed_ || !gps_received_) return;
        ROS_INFO("First Scan Sub !"); 
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_msg, *current_cloud);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(-current_roll_, Eigen::Vector3f::UnitX())); // Roll rotation
        transform.rotate(Eigen::AngleAxisf(-current_pitch_, Eigen::Vector3f::UnitY())); // Pitch rotation
        transform.rotate(Eigen::AngleAxisf(-current_yaw_, Eigen::Vector3f::UnitZ())); // Yaw rotation
        pcl::transformPointCloud(*current_cloud, *current_cloud, transform);

        // gps값을 비교해서 최초 initial x, y값 생성
        match_x = (_longitude - match_x) * 111320 * cos(_latitude * M_PI / 180.0);  // longitude 변환 (cos 적용)
        match_y = (_latitude - match_y) * 111320;  // latitude 변환
        match_z = 0.0; // 고도는 0으로 설정
        ROS_INFO("Initial Using GPS x, y : %f, %f", match_x, match_y);
        
        Eigen::Affine3f translated_transform = Eigen::Affine3f::Identity();
        translated_transform.translate(Eigen::Vector3f {match_x, match_y, match_z});       
        pcl::PointCloud<pcl::PointXYZI>::Ptr translated_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*current_cloud, *translated_cloud, translated_transform);

        std::vector<Eigen::Vector3f> directions = {
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.2},
            {0.0f, 0.0f, 0.4},
            {0.0f, 0.0f, 0.6},
            {0.0f, 0.0f, 0.8},
            {0.0f, 0.0f, 1.0},
            {0.0f, 0.0f, -0.2},
            {0.0f, 0.0f, -0.4},
            {0.0f, 0.0f, -0.6},
            {0.0f, 0.0f, -0.8},
            {0.0f, 0.0f, -1.0},
        };
        
        float best_score = std::numeric_limits<float>::max();
        Eigen::Matrix4f best_transformation = Eigen::Matrix4f::Identity();
        Eigen::Vector3f best_direction;

        for (const auto& direction : directions) {
            // 지정한 direction들에 대해서 시작위치 initial
            Eigen::Affine3f rotation = Eigen::Affine3f::Identity();
            rotation.rotate(Eigen::AngleAxisf(direction[0], Eigen::Vector3f::UnitX())); // Roll rotation
            rotation.rotate(Eigen::AngleAxisf(direction[1], Eigen::Vector3f::UnitY())); // Pitch rotation
            rotation.rotate(Eigen::AngleAxisf(direction[2], Eigen::Vector3f::UnitZ())); // Yaw rotation

            pcl::PointCloud<pcl::PointXYZI>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*current_cloud, *rotated_cloud, rotation);

            pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;

            gicp.setInputSource(rotated_cloud);
            gicp.setInputTarget(global_map_.makeShared());
            gicp.setMaxCorrespondenceDistance(1.0);
            gicp.setTransformationEpsilon(0.001);
            gicp.setMaximumIterations(1000);

            gicp.align(*aligned_cloud);

            if (gicp.hasConverged()) {
                float score = gicp.getFitnessScore();
                ROS_INFO("GICP Score at direction [R=%f, P=%f, Y=%f]: %f",
                         direction[0], direction[1], direction[2], score);

                if (score < best_score) {
                    best_direction = direction;
                    best_score = score;
                    best_transformation = gicp.getFinalTransformation();
                }
            } else {
                ROS_WARN("GICP did not converge for direction [R=%f, P=%f, Y=%f].",
                         direction[0], direction[1], direction[2]);
            }
        }

        ROS_INFO("Best GICP Score: %f", best_score);

        Eigen::Matrix3f rotation_matrix = best_transformation.block<3, 3>(0, 0);
        Eigen::Vector3f translation_vector = best_transformation.block<3, 1>(0, 3);
        Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(0, 1, 2);

        ROS_INFO("Best Transformation: [Roll, Pitch, Yaw, X, Y, Z]: [%f, %f, %f, %f, %f, %f]",
                 euler_angles[0], euler_angles[1], euler_angles[2],
                 translation_vector[0], translation_vector[1], translation_vector[2]);

        ROS_INFO("And you can use this for x, y, z R P Y: %f %f %f %f %f %f",
                match_x + translation_vector[0], 
                match_y + translation_vector[1],
                          translation_vector[2],
                -current_roll_ + best_direction[0],
                -current_pitch_ + best_direction[1],
                -current_yaw_ + best_direction[2]
                );

        ros::shutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;

    pcl::PointCloud<pcl::PointXYZI> global_map_;

    bool processed_ = false; 
    bool imu_received_ = false;
    bool gps_received_ = false;
    float current_roll_ = 0.0;
    float current_yaw_ = 0.0;
    float current_pitch_ = 0.0; 
    float _latitude, _longitude, _altitude;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_mapping_node");
    LidarMappingNode node;
    ros::spin();
    return 0;
}
