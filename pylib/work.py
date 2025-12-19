import os
import json
import socket
import struct
import logging
import numpy as np
import open3d as o3d
from datetime import datetime

from pylib.utils import filter_points_within_square, extract_plane_points, extract_non_floor_plane_points
from pylib.utils import downsample_points, compute_metrics_with_grid
from pylib.utils import upload_data_to_reporting_server, upload_file_to_reporting_server
from pylib.misc import generate_stamp, COLORS_MAP

logger = logging.getLogger()

SMOOTHING_WEIGHTS = {
    1: np.array([1.0]),
    2: np.array([0.4, 0.6]),
    3: np.array([0.2, 0.3, 0.5]),
    4: np.array([0.1, 0.2, 0.3, 0.4]),
    5: np.array([0.1, 0.15, 0.2, 0.25, 0.3]),
    6: np.array([0.1, 0.12, 0.15, 0.18, 0.2, 0.25]),
}

def gather_point_cloud(args, manager, pcd_stamp=''):
    """ Gather point cloud data from the Lidar.

    Args:
        args (argparse.Namespace): Parsed command line arguments.
        manager (lidar.LidarManager): Lidar manager instance.
        pcd_stamp (str): Optional stamp for the point cloud data.
    """
    ## get point cloud data from Lidar
    pcd = o3d.geometry.PointCloud()
    for i in range(args.gather_times):
        logger.info(f"Gathering point cloud data {i + 1}/{args.gather_times}...")
        raw_points = manager.getPointCloudBatch(args.point_batch)
        raw_points = np.array(raw_points, dtype=np.float64)[:, :4]

        xyz = raw_points[:, :3]
        xyz = np.ascontiguousarray(xyz, dtype=np.float64)
        intensity = raw_points[:, 3]

        norm_i = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity) + 1e-8)
        norm_i = (norm_i * 191).astype(np.uint8)
        colors = COLORS_MAP[norm_i] / 255.0
        colors = np.ascontiguousarray(colors, dtype=np.float64)

        batch_pcd = o3d.geometry.PointCloud()
        batch_pcd.points = o3d.utility.Vector3dVector(xyz)
        batch_pcd.colors = o3d.utility.Vector3dVector(colors)

        ## filter xyz first.
        batch_pcd = filter_points_within_square(
            batch_pcd,
            length=args.space_region_threshold,
            below_lidar_threshold=args.lidar_height_threshold
        )

        ## Merge into global point cloud, and then
        ## downsample the points, as the density of the points is uniform
        pcd = pcd + batch_pcd
        pcd = downsample_points(pcd, voxel_size=0.02)

    ## save points if needed
    if args.save_point_cloud:
        pcd_stamp = generate_stamp(random_sufix_length=0) if pcd_stamp == '' else pcd_stamp
        save_path = os.path.join(args.PCDS_FOLDER, args.stamp, f"{args.device_id}-{pcd_stamp}.pcd")
        o3d.io.write_point_cloud(save_path, pcd)

    return pcd

def update_lowest_height(plane_points, args):
    """ Update the lowest height (floor height) based on the plane points.

    Args:
        plane_points (np.ndarray): Points that are likely part of the plane.
        args (argparse.Namespace): Parsed command line arguments.
    """
    ## compute the lowest height from the plane points
    ## As the z+ axis is downwards, the lower point has larger z value
    ## Importantly, the floor must possess at least 10% of the plane points,
    ## we use the 90th percentile of the height to determine the floor points
    z = plane_points[:, 2]
    anchor_z = float(round(np.percentile(z, 90), 3))
    floor_points = plane_points[np.abs(z - anchor_z) < args.floor_height_threshold]

    floor = o3d.geometry.PointCloud()
    floor.points = o3d.utility.Vector3dVector(floor_points)

    ## fit a plane to these floor points
    plane_model, inliers = floor.segment_plane(
        distance_threshold=args.floor_height_threshold / 2,
        ransac_n=3,
        num_iterations=len(floor_points) * 2
    )
    a, b, c, d = plane_model
    x_0_y_0_z = -d / c
    normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])

    angle = np.arccos(np.dot(normal, np.array([0, 0, 1]))) * 180 / np.pi
    logger.info(f"Detected lowest height (floor height): {x_0_y_0_z:.6f} m, angle with vertical: {angle:.3f} degrees.")
    return x_0_y_0_z

def create_colored_plane_points(plane_points, floor_height, alert_height, height_scale,
                                      floor_height_threshold):
    """Create a planar point cloud: based on height shading, super high points and ground points are marked in red

    Args:
        plane_points (np.ndarray): Array of points.
        floor_height (float): Height of the floor plane.
        alert_height (float): Alarm height (m), used to determine the quadrant
        height_scale (float): Scale factor for the computed height.
        floor_height_threshold (float): Threshold to determine if points are part of the floor.
    """
    # Create a point cloud object
    plane_pcd = o3d.geometry.PointCloud()

    if len(plane_points) == 0:
        plane_pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))
        plane_pcd.colors = o3d.utility.Vector3dVector(np.zeros((1, 3)))
        return plane_pcd

    plane_pcd.points = o3d.utility.Vector3dVector(plane_points)

    heights = floor_height - plane_points[:, 2]  # Ground height - point height=actual height
    scaled_heights = heights * height_scale

    # Identify different types of points
    n_points = len(plane_points)
    colors = np.zeros((n_points, 3), dtype=np.float32)

    # Mark ground points
    ground_mask = plane_points[:, 2] >= (floor_height - floor_height_threshold)

    # Mark the ultra-high point
    high_alert_mask = scaled_heights > alert_height

    # Mark points at normal height
    normal_mask = ~ground_mask & ~high_alert_mask

    # Coloring points based on height for normal height
    if np.any(normal_mask):
        normal_heights = scaled_heights[normal_mask]

        # Normalize the height only for points of normal height
        if len(normal_heights) > 0:
            min_normal_height = np.min(normal_heights)
            max_normal_height = np.max(normal_heights)

            if max_normal_height > min_normal_height:
                normalized_heights = (normal_heights - min_normal_height) / (max_normal_height - min_normal_height)

                # R: 0-0.5, G: 0-0.7, B: 0.5-1.0
                blue_colors = np.zeros((len(normalized_heights), 3))
                # r
                blue_colors[:, 0] = normalized_heights * 0.3
                # g
                blue_colors[:, 1] = normalized_heights * 0.7
                # b
                blue_colors[:, 2] = 0.5 + normalized_heights * 0.5

                colors[normal_mask] = blue_colors
            else:
                # If all normal points have the same height, use medium blue
                colors[normal_mask] = [0.2, 0.4, 0.8]

    # Ground points marked in red
    if np.any(ground_mask):
        colors[ground_mask] = [1, 0, 1]

    # The ultra-high point is marked in red
    if np.any(high_alert_mask):
        colors[high_alert_mask] = [1, 0, 0]

    # Set point cloud color
    plane_pcd.colors = o3d.utility.Vector3dVector(colors)

    return plane_pcd

def workflow(args, manager, history):
    """ Main workflow for processing point cloud data.

    Args:
        args (argparse.Namespace): Parsed command line arguments.
        manager (lidar.LidarManager): Lidar manager instance.
        history (collections.deque): History of previous results for smoothing.
    """
    all_cargo_points = []  # Store collection_times_per_cycle batches of goods in a flat point cloud
    batch_lowest_heights = []
    ## gather the point cloud data from the Lidar
    for i in range(args.collection_times_per_cycle):
        logger.info(f"colloection_{i + 1}/{args.collection_times_per_cycle} ...")

        pcd = gather_point_cloud(args, manager)
        if pcd is None or len(pcd.points) == 0:
            logger.warning("No points gathered. Lidar may not be working properly.")
            continue
        points_np = np.asarray(pcd.points)

        ## extract plane points
        plane_points = extract_plane_points(
            points=points_np,
            degrees_threshold=args.normal_degrees_threshold,
        )
        if len(plane_points) < 3:
            logger.warning("Not enough points to form a plane. Lidar maybe not working properly.")
            continue

        ## update lowest height if needed
        ## default, we use the configured lowest height
        ## if update_lowest_height is True, we use the updated the lowest height based on the plane points
        ## we always use the smoothed lowest height for further processing
        lowest_z = args.lowest_height
        if args.update_lowest_height:
            lowest_z = update_lowest_height(plane_points, args)
            batch_lowest_heights.append(lowest_z)
            weight = SMOOTHING_WEIGHTS[len(history) + 1]
            lowest_z = np.sum(np.array([item['lowest_z'] for item in history] + [lowest_z]) * weight) / (
                        np.sum(weight) + 1e-8)
        else:
            batch_lowest_heights.append(lowest_z)

        logger.info(f"Using lowest height (floor height) of {lowest_z:.6f} m for further processing.")

        ## remove points that are likely part of the floor plane
        non_floor_plane_points = extract_non_floor_plane_points(
            points=plane_points,
            lowest_height=lowest_z,
            floor_height_threshold=args.floor_height_threshold
        )

        if len(non_floor_plane_points) > 0:
            all_cargo_points.append(non_floor_plane_points)

    if not all_cargo_points:
        return None

    final_lowest_z = np.mean(batch_lowest_heights)

    ## compute volume, area, and height from these point groups
    volume, area, max_height, mean_height, quadrant = compute_metrics_with_grid(
        all_cargo_points,
        floor_height=final_lowest_z,
        grid_size=args.grid_size,
        alert_height=args.alert_height,
        area_scale = args.area_scale,
        height_scale = args.height_scale,
        min_valid_collections=args.min_valid_collections
    )

    logger.info(f"Raw results from grid method: Volume = {volume + args.volume_adjustment:.6f} m^3, Area = {area:.6f} m^2, Max Height = {max_height:.6f} m, Mean Height = {mean_height:.6f} m, Quadrant = {quadrant}.")

    ## smooth results with history
    weight = SMOOTHING_WEIGHTS[len(history) + 1]
    volume = np.sum(np.array([item['volume'] for item in history] + [volume]) * weight) / (np.sum(weight) + 1e-8)
    area = np.sum(np.array([item['area'] for item in history] + [area]) * weight) / (np.sum(weight) + 1e-8)
    max_height = np.sum(np.array([item['max_height'] for item in history] + [max_height]) * weight) / (np.sum(weight) + 1e-8)
    mean_height = np.sum(np.array([item['mean_height'] for item in history] + [mean_height]) * weight) / (np.sum(weight) + 1e-8)

    # Merge t batch point clouds and downsample
    merged_cargo_points = np.vstack(all_cargo_points)  # Merge the point cloud of batch T goods on the plane
    merged_pcd = o3d.geometry.PointCloud()
    merged_pcd.points = o3d.utility.Vector3dVector(merged_cargo_points)
    downsampled_pcd = downsample_points(merged_pcd, voxel_size=0.02)

    plane_pcd = create_colored_plane_points(
        np.asarray(downsampled_pcd.points),
        floor_height=final_lowest_z,
        alert_height=args.alert_height,
        height_scale=args.height_scale,
        floor_height_threshold=args.floor_height_threshold
    )

    results = {
        'volume': max(0.0, volume + args.volume_adjustment),
        'area': area,
        'max_height': max_height,
        'mean_height': mean_height,
        'lowest_z': final_lowest_z,
        'quadrant': str(quadrant) if quadrant > 0 else '',
        'points': plane_pcd
    }
    return results

def send_results_to_reporting_server(args, results):
    """ Send results to the reporting server.
    Args:
        args (argparse.Namespace): Parsed command line arguments.
        results (dict): Results from the workflow.
    """
    send_data = {
        'companyCode': args.company_id,
        'companyName': args.company_name,
        'equipmentNum': args.device_id,
        'equipmentName': args.device_name,
        'volume': float(results['volume']),
        'averageHeight': float(results['mean_height']),
        'height': float(results['max_height']),
        'quadrant': results['quadrant'],
        'url': '',
        'createTime': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
    }

    ## prepare pcd/image of points and upload it to the server
    pcd = results['points']
    npoints = len(pcd.points)
    if args.points_for_uploading > 0 and args.points_for_uploading < npoints:
        indices = np.random.choice(npoints, args.points_for_uploading, replace=False)
        pcd = pcd.select_by_index(indices)
    o3d.io.write_point_cloud(args.POINTS_SAVE_FILE, pcd)

    ## Update image/text of pc to reporting server
    if args.upload_file:
        res, pc_url = upload_file_to_reporting_server(
            args=args,
            file_path=args.POINTS_SAVE_FILE,
        )
        send_data['url'] = pc_url # update url
        if res:
            logger.info(f"File {args.POINTS_SAVE_FILE} uploaded to reporting server. Returned URL is {pc_url}.")
    else:
        logger.info("Skipping file upload to reporting server.")

    ## Upload data to reporting server
    if args.upload_data:
        res = upload_data_to_reporting_server(
            args=args,
            data=send_data,
        )
        if res:
            logger.info(f"Below json data uploaded to reporting server.\n{json.dumps(send_data, indent=2, ensure_ascii=False)}")
    else:
        logger.info("Skipping data upload to reporting server.")

def send_results_to_visualization_server(args, results):
    """ Send results to the visualization server.

    Args:
        args (argparse.Namespace): Parsed command line arguments.
        results (dict): Results from the workflow.
    """
    HOST = args.v_server_address
    PORT = args.v_server_port

    volume = results['volume']
    area = results['area']
    max_height = results['max_height']
    mean_height = results['mean_height']
    lowest_z = results['lowest_z']
    pcd = results['points']
    points = np.hstack([
        np.asarray(pcd.points),
        np.asarray(pcd.colors)
    ]).astype(np.float32)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))

            ## point data
            data_shape = points.shape
            data_bytes = points.tobytes()

            ## meta data
            meta = struct.pack('>fffff', volume, area, max_height, mean_height, lowest_z)
            meta += struct.pack('>II', data_shape[0], data_shape[1])

            ## send meta and point data
            s.sendall(struct.pack('>I', len(meta)))
            s.sendall(meta)
            s.sendall(struct.pack('>I', len(data_bytes)))
            s.sendall(data_bytes)

            logger.info(f"Sent {data_shape[0]} points to visualization server {HOST}:{PORT}.")
    except Exception as e:
        logger.error(f"Failed to send data: {e}")