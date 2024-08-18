#include "utility.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
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
        ROS_INFO("\033[1;33m----> Loading for Map . . .\033[0m");
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(PathGlobalMap, global_map_) == -1)
        {
            ROS_ERROR("Couldn't read the global map PCD file");
            ros::shutdown();
            return;
        }
        ROS_INFO("\033[1;34mGlobal map loaded with %zu points ! ! \033[0m", global_map_.points.size());
    }


    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        if(imu_received_) return;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(imu_msg->orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
        imu_received_ = true;
    }


    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        // 첫 번째 포인트 클라우드 처리 후 노드 종료
        if (processed_) return;
        ROS_INFO("First Scan Sub !");
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_msg, *current_cloud);

        // current_cloud를 yaw값을 받아와서 -만큼 rotation 준 다음 gicp 진행
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(-current_yaw_, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud(*current_cloud, *current_cloud, transform);

        std::vector<Eigen::Vector3f> directions = {
            { match_x,  match_y, 0.0f},  
            { match_x + 0.3,  match_y, 0.0f},  
            { match_x,  match_y + 0.1, 0.0f},  
            { match_x + 0.3, match_y + 0.1, 0.0f},  
            { match_x + 0.5,  match_y, 0.0f},  
            { match_x,  match_y + 0.1, 0.0f},  
            { match_x + 0.5, match_y + 0.1, 0.0f},  
        };

        float best_score = std::numeric_limits<float>::max();
        Eigen::Matrix4f best_transformation = Eigen::Matrix4f::Identity();

        for (const auto& direction : directions) {
            // 지정한 direction들에 대해서 시작위치 initial
            Eigen::Affine3f translated_transform = Eigen::Affine3f::Identity();
            translated_transform.translate(direction);

            pcl::PointCloud<pcl::PointXYZI>::Ptr translated_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*current_cloud, *translated_cloud, translated_transform);

            pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;

            gicp.setInputSource(translated_cloud);
            gicp.setInputTarget(global_map_.makeShared());
            gicp.setMaxCorrespondenceDistance(0.7);
            gicp.setTransformationEpsilon(0.001);
            gicp.setMaximumIterations(1000);

            gicp.align(*aligned_cloud);

            if (gicp.hasConverged()) {
                float score = gicp.getFitnessScore();
                ROS_INFO("GICP Score at direction [x=%f, y=%f, z=%f]: %f",
                         direction[0], direction[1], direction[2], score);

                if (score < best_score) {
                    best_score = score;
                    best_transformation = gicp.getFinalTransformation();
                }
            } else {
                ROS_WARN("GICP did not converge for direction [x=%f, y=%f, z=%f].",
                         direction[0], direction[1], direction[2]);
            }
        }

        ROS_INFO("Best GICP Score: %f", best_score);
        ROS_INFO_STREAM("Best Transformation Matrix:\n" << best_transformation);

        Eigen::Matrix3f rotation_matrix = best_transformation.block<3, 3>(0, 0);
        Eigen::Vector3f translation_vector = best_transformation.block<3, 1>(0, 3);
        Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(0, 1, 2);

        ROS_INFO("Best Transformation: [Roll, Pitch, Yaw, X, Y, Z]: [%f, %f, %f, %f, %f, %f]",
                 euler_angles[0], euler_angles[1], euler_angles[2],
                 translation_vector[0], translation_vector[1], translation_vector[2]);
        ros::shutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;

    pcl::PointCloud<pcl::PointXYZI> global_map_;

    bool processed_ = false; 
    bool imu_received_ = false;
    double current_yaw_ = 0.0;
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_mapping_node");
    LidarMappingNode node;
    ros::spin();
    return 0;
}
