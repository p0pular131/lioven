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

/*
    1. 먼저 yaw값을 바탕으로 curr_scan을 map frame으로 rotation을 대충 맞춰준다. (-yaw 값만큼 회전) 
    2. 각각의 position에 대해서 변환행렬을 구하고, score를 바탕으로 가장 괜찮은거 선정. (0.1이하로 나와야 쓸만함)

*/


class LidarMappingNode
{
public:
    LidarMappingNode()
    {
        // LiDAR 포인트 클라우드 데이터를 구독
        lidar_sub_ = nh_.subscribe("/velodyne_points", 1, &LidarMappingNode::lidarCallback, this);
        imu_sub_ = nh_.subscribe("/imu", 1, &LidarMappingNode::imuCallback, this);
        // Global map을 PCD 파일에서 로드
        if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/popular/catkin_ws/src/lioven/data/cloudGlobal.pcd", global_map_) == -1)
        {
            ROS_ERROR("Couldn't read the global map PCD file");
            ros::shutdown();
            return;
        }
        ROS_INFO("Global map loaded with %zu points", global_map_.points.size());
    }


    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        if(imu_received_) return;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(imu_msg->orientation, quat);
        double roll, pitch;
        tf::Matrix3x3(quat).getRPY(roll, pitch, current_yaw_);
        imu_received_ = true;
    }


    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        // 첫 번째 포인트 클라우드 처리 후 노드 종료
        if (processed_) return;
        ROS_INFO("First Scan Sub !");
        // PCL 포맷으로 변환
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_msg, *current_cloud);

        // current_cloud를 yaw값을 받아와서 -만큼 rotation 준 다음 gicp 진행
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(-current_yaw_, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud(*current_cloud, *current_cloud, transform);

        std::vector<Eigen::Vector3f> directions = {
            { 0.5f,  0.0f, 0.0f},  // East
            {-0.5f,  0.0f, 0.0f},  // West
            { 0.0f,  0.5f, 0.0f},  // North
            { 0.0f, -0.5f, 0.0f},  // South
            { 0.5f,  0.5f, 0.0f},  // North-East
            {-0.5f,  0.5f, 0.0f},  // North-West
            { 0.5f, -0.5f, 0.0f},  // South-East
            {-0.5f, -0.5f, 0.0f},  // South-West
            { 0.0f,  0.0f, 0.0f}   // Original position
        };

        float best_score = std::numeric_limits<float>::max();
        Eigen::Matrix4f best_transformation = Eigen::Matrix4f::Identity();

        for (const auto& direction : directions) {
            // Translate the point cloud
            Eigen::Affine3f translated_transform = Eigen::Affine3f::Identity();
            translated_transform.translate(direction);
            pcl::PointCloud<pcl::PointXYZI>::Ptr translated_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*current_cloud, *translated_cloud, translated_transform);

            // Perform GICP alignment
            pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
            gicp.setInputSource(translated_cloud);
            gicp.setInputTarget(global_map_.makeShared());
            gicp.setMaxCorrespondenceDistance(2.0);
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

        // Output the best transformation and its corresponding score
        ROS_INFO("Best GICP Score: %f", best_score);
        ROS_INFO_STREAM("Best Transformation Matrix:\n" << best_transformation);

        // Extract roll, pitch, yaw from the best transformation matrix
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
    double current_yaw_;
    bool imu_received_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_mapping_node");
    LidarMappingNode node;
    ros::spin();
    return 0;
}
