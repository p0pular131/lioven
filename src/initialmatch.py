#!/usr/bin/env python3

import rospy
import pcl
import sensor_msgs.point_cloud2 as pc2
import small_gicp
import numpy as np
import open3d as o3d
import copy
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
from tqdm import tqdm

class PointCloudSaverAndMatcher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('pcd_saver_and_matcher', anonymous=True)

        # Subscribe to the PointCloud2 topic
        self.point_cloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.point_cloud_callback)

        # Directory to save PCD files
        self.pcd_save_path = "first_scan.pcd"

        self.global_map_path = rospy.get_param("lioven/PathGlobalMap")
        self.key_pose_path = rospy.get_param("lioven/KeyPosePath")
        self.use_odom_pose = rospy.get_param("lioven/useOdom")

        self.pose_x = rospy.get_param("lioven/position/x")
        self.pose_y = rospy.get_param("lioven/position/y")
        self.pose_z = rospy.get_param("lioven/position/z")
        self.orientation_x = rospy.get_param("lioven/orientation/x")
        self.orientation_y = rospy.get_param("lioven/orientation/y")
        self.orientation_z = rospy.get_param("lioven/orientation/z")
        self.orientation_w = rospy.get_param("lioven/orientation/w")

        # Global map and key poses for matching
        self.global_map_np = self.read_pcd_file(self.global_map_path)
        if self.global_map_np is None:
            rospy.logerr("Failed to load the global map. Shutting down the node.")
            rospy.signal_shutdown("Error in global map loading.")
        self.key_poses = self.read_kitty_format_poses(self.key_pose_path)

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
        if self.use_odom_pose :
            self.match_point_cloud_odom()
        else :
            self.match_point_cloud()

        # After matching, shutdown ROS node
        rospy.signal_shutdown("Matching completed.")

    def read_pcd_file(self, pcd_file_path):
        try:
            pcd = o3d.io.read_point_cloud(pcd_file_path)
            points = np.asarray(pcd.points)
            if points.size == 0:
                rospy.logerr(f"PCD file is empty: {pcd_file_path}")
                return None
            return points
        except Exception as e:
            rospy.logerr(f"Failed to read PCD file: {e}")
            return None

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

    def odom_to_transform_matrix(self, pose_position, pose_orientation):
        x, y, z = pose_position
        qx, qy, qz, qw = pose_orientation
        
        rotation_matrix = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = [x, y, z]
        
        return transform_matrix


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

        for i in tqdm(range(len(self.key_poses))):
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
        
    def match_point_cloud_odom(self) :
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
        pose = (self.pose_x, self.pose_y, self.pose_z)
        orientation = (self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w)
        key_pose_odom = self.odom_to_transform_matrix(pose, orientation)
        expanded_key_poses = []
        for j in range(360):
            rotated_key_pose = np.copy(key_pose_odom)
            R = Rotation.from_euler('z', j*1, degrees=True)
            rotated_key_pose[:3,:3] = R.as_matrix() @ rotated_key_pose[:3,:3]
            expanded_key_poses.append(rotated_key_pose)

        for i in tqdm(range(len(expanded_key_poses))):
            key_pose = expanded_key_poses[i]
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