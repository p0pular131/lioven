#!/usr/bin/env python3

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PointCloudSaver:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('pcd_saver', anonymous=True)

        # Subscribe to the PointCloud2 topic
        self.point_cloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.point_cloud_callback)

        # Directory to save PCD files
        self.pcd_save_path = "../data"  # Adjust this path

        # Initialize counter for filenames
        self.counter = 0

    def point_cloud_callback(self, msg):
        # Convert ROS PointCloud2 message to PCL PointCloud
        cloud_points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))

        # Create PCL PointCloud object
        pcl_cloud = pcl.PointCloud()
        pcl_cloud.from_list(cloud_points)

        # Define filename with counter
        pcd_filename = "first_scan.pcd"

        # Save the point cloud to a PCD file
        pcl.save(pcl_cloud, pcd_filename)

        # Increment counter for next file
        self.counter += 1

        rospy.loginfo(f"Saved {pcd_filename}")

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        saver = PointCloudSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
