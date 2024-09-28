#!/usr/bin/env python3

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import small_gicp
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

class PointCloudSaverAndMatcher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('pcd_saver_and_matcher', anonymous=True)

        # Subscribe to the PointCloud2 topic
        self.point_cloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.point_cloud_callback)

        # Directory to save PCD files
        self.pcd_save_path = "first_scan.pcd"

        # Global map and key poses for matching
        self.global_map_np = self.read_pcd_file("../data/bonsun/cloudGlobal.pcd")
        self.key_poses = self.read_kitty_format_poses("../data/bonsun/optimized_poses.txt")

    def point_cloud_callback(self, msg):
        # Convert ROS PointCloud2 message to PCL PointCloud
        cloud_points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))

        # Create PCL PointCloud object
        pcl_cloud = pcl.PointCloud()
        pcl_cloud.from_list(cloud_points)

        # Save the point cloud to a PCD file
        pcl.save(pcl_cloud, self.pcd_save_path)
        rospy.loginfo(f"Saved {self.pcd_save_path}")

        # Once the point cloud is saved, proceed to match it
        self.match_point_cloud()

        # After matching, shutdown ROS node
        rospy.signal_shutdown("Matching completed.")

    def read_pcd_file(self, pcd_file_path):
        pcd = o3d.io.read_point_cloud(pcd_file_path)
        points = np.asarray(pcd.points)
        return points

    def read_kitty_format_poses(self, path):
        with open(path, "r") as txt_file:
            lines = txt_file.readlines()
            poses = []
            for line in lines:
                pose = np.eye(4)
                l2w = np.array(list(map(float, line.split()))).reshape(3, 4)
                pose[:3,:4] = l2w
                poses.append(pose)
            return poses

    def match_point_cloud(self):
        # Current scan
        current_scan = self.read_pcd_file(self.pcd_save_path)

        # Finding optimal initial pose using G-ICP
        raw_target_points = small_gicp.PointCloud(self.global_map_np)
        target_points = small_gicp.voxelgrid_sampling(raw_target_points, 0.2)
        target_tree = small_gicp.KdTree(target_points)
        small_gicp.estimate_covariances(target_points, target_tree, 25, 64)

        raw_source_points = small_gicp.PointCloud(current_scan)
        source_points = small_gicp.voxelgrid_sampling(raw_source_points, 0.2)
        source_tree = small_gicp.KdTree(source_points)
        small_gicp.estimate_covariances(source_points, source_tree, 25, 64)

        errors = []
        est_poses = []
        h = np.ones((current_scan.shape[0], 1))
        current_scan = np.concatenate((current_scan, h), axis=1)

        for i in range(len(self.key_poses)):
            key_pose = self.key_poses[i]
            result = small_gicp.align(target=target_points,
                                      source=source_points,
                                      target_tree=target_tree,
                                      init_T_target_source=key_pose,
                                      registration_type="GICP",
                                      max_correspondence_distance=1.0,
                                      num_threads=16,
                                      max_iterations=20)
            est_poses.append(result.T_target_source)
            errors.append(result.error / result.num_inliers)

        opt_idx = np.argmin(errors)
        est_pose = est_poses[opt_idx]

        # Print estimated optimal initial pose
        R = Rotation.from_matrix(est_pose[:3, :3])
        x, y, z = est_pose[0, 3], est_pose[1, 3], est_pose[2, 3]
        euler = R.as_euler('xyz')
        print(f"intial_x: {x}\n" 
            f"intial_y: {y}\n" 
            f"intial_z: {z}\n" 
            f"intial_roll: {euler[0]}\n" 
            f"intial_pitch: {euler[1]}\n" 
            f"intial_yaw: {euler[2]}")

if __name__ == '__main__':
    try:
        matcher = PointCloudSaverAndMatcher()
        rospy.spin()  # Node will stop after the first match
    except rospy.ROSInterruptException:
        pass