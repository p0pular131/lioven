import small_gicp
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import copy
from tqdm import tqdm

def read_pcd_file(pcd_file_path):

    pcd = o3d.io.read_point_cloud(pcd_file_path)
    points = np.asarray(pcd.points)
    
    return points

def read_kitty_format_poses(path):
    txt_file = open(path, "r")
    lines = txt_file.readlines()
    poses = []
    
    for line in lines:
        pose = np.eye(4)
        l2w = np.array(list(map(float, line.split()))).reshape(3, 4)
        pose[:3,:4] = l2w
        poses.append(pose)
    
    return poses

if __name__ == "__main__":
    # load global map
    global_map_np = read_pcd_file("../data/cloudGlobal.pcd")
    # key poses, will be used as candidate initial poses
    key_poses = read_kitty_format_poses("../data/optimized_poses.txt")
    # current scan
    current_scan = read_pcd_file("first_scan.pcd")

    # finding optimal initial pose (for localization module)
    ## target points (map point cloud)
    raw_target_points = small_gicp.PointCloud(global_map_np)
    target_points = small_gicp.voxelgrid_sampling(raw_target_points, 0.2)
    target_tree = small_gicp.KdTree(target_points)
    small_gicp.estimate_covariances(target_points, target_tree, 25, 64)
    ## source points (current scan point cloud)
    raw_source_points = small_gicp.PointCloud(current_scan)
    source_points = small_gicp.voxelgrid_sampling(raw_source_points, 0.2)
    source_tree = small_gicp.KdTree(source_points)
    small_gicp.estimate_covariances(source_points, source_tree, 25, 64)
    
    ## align
    ### expand candidate poses, by rotating current scan 
    # expanded_key_poses = []
    # for i in range(len(key_poses)):
    #     key_pose = key_poses[i]
    #     for j in range(4):
    #         rotated_key_pose = copy.deepcopy(key_pose)
    #         R = Rotation.from_euler('z', j*3, degrees=True)
    #         rotated_key_pose[:3,:3] = R.as_matrix() @ rotated_key_pose[:3,:3]
    #         expanded_key_poses.append(rotated_key_pose)
    #     for j in range(4):
    #         rotated_key_pose = copy.deepcopy(key_pose)
    #         R = Rotation.from_euler('z', j*-3, degrees=True)
    #         rotated_key_pose[:3,:3] = R.as_matrix() @ rotated_key_pose[:3,:3]
    #         expanded_key_poses.append(rotated_key_pose)
        
    errors = []
    est_poses = []
    
    # for visualization
    h = np.ones((current_scan.shape[0],1))
    current_scan = np.concatenate((current_scan, h), axis=1)
    
    ### align current scan to the map using G-ICP
    ### expanded candidate initial poses will be used as initial guess for G-ICP
    for i in tqdm(range(len(key_poses))):
        key_pose = key_poses[i]    # normal version
    # for i in tqdm(range(len(expanded_key_poses))): 
    #     key_pose = expanded_key_poses[i]    # extended version initial guess for G-ICP
        result = small_gicp.align(target=target_points,
                                source=source_points,
                                target_tree=target_tree,
                                init_T_target_source=key_pose,
                                registration_type="GICP",
                                max_correspondence_distance=1.0,
                                num_threads=16,
                                max_iterations=20)
        est_poses.append(result.T_target_source)
        errors.append(result.error/result.num_inliers)
        transformed_current_scan = (result.T_target_source@current_scan.T).T[:,:3]

    
    opt_idx = np.argmin(errors)
    est_pose = est_poses[opt_idx]
    
    # print estimated optimal initial pose
    R = Rotation.from_matrix(est_pose[:3,:3])
    x = est_pose[0,3]
    y = est_pose[1,3]
    z = est_pose[2,3]
    euler = R.as_euler('xyz')
    print(f"result: {x} {y} {z} {euler[0]} {euler[1]} {euler[2]}")