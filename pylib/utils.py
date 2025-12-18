import logging
import requests
import numpy as np
import open3d as o3d
from scipy.spatial import Delaunay, cKDTree

from pylib.misc import COLORS_MAP

try:
    import djset
except ImportError:
    try:
        import os, sys

        sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'build'))
        import djset
    except ImportError:
        print("Failed to import the djset module. Please ensure the build path is imported correctly.")
        sys.exit(1)

logger = logging.getLogger()


## [Filter]
def filter_points_within_square(pcd, length=2.0, below_lidar_threshold=0.1):
    """ Filter points that are within a certain square from the origin.

    Args:
        pcd: (open3d.geometry.PointCloud).
        length (float): Side length of the square to filter points within.
        below_lidar_threshold (float): Threshold to ensure points are below the lidar.
    """
    if pcd is None or len(pcd.points) == 0:
        return pcd

    ## remove points that are outside the square region
    points = np.asarray(pcd.points)
    mask = (
            (np.abs(points[:, 0]) < length) &
            (np.abs(points[:, 1]) < length) &
            (points[:, 2] > below_lidar_threshold)
    )

    temp_pcd = o3d.geometry.PointCloud()
    temp_pcd.points = o3d.utility.Vector3dVector(points[mask])

    # Retain color information (if present)
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)[mask]
        temp_pcd.colors = o3d.utility.Vector3dVector(colors)

    ## remove the some bad points
    new_pcd = temp_pcd.remove_non_finite_points()
    new_pcd = new_pcd.remove_duplicated_points()

    return new_pcd


## [Filter]
def extract_plane_points(points, degrees_threshold=5.0):
    """ Extract plane points from the point cloud data.

    Args:
        points (np.ndarray): Array of points with shape (N, 3).
        degrees_threshold (float): Threshold in degrees for normal vector alignment.
    """
    points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

    ## Remove outliers
    points, _ = points.remove_radius_outlier(nb_points=2, radius=0.1)
    # points, _ = points.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    ## Estimate normals
    points.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=66))
    normals = np.asarray(points.normals)  # (N, 3)

    ## Filter points based on normals
    z_axis = np.array([0, 0, 1], dtype=np.float64)
    dot_products = normals @ z_axis

    indices = np.where(np.abs(dot_products) > np.cos(np.pi / 180 * degrees_threshold))[0]
    points = points.select_by_index(indices)

    # Remove outliers
    points, _ = points.remove_radius_outlier(nb_points=2, radius=0.1)
    # points, _ = points.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    return np.asarray(points.points)


## [Filter]
def extract_non_floor_plane_points(points, lowest_height, floor_height_threshold=0.1):
    """ Remove points that are likely part of the floor plane.

    Args:
        points (np.ndarray): Points that are likely part of the plane.
        lowest_height (float): Height of the floor plane.
        floor_height_threshold (float): Threshold to determine if points are part of the floor.
    """
    ## remove points that are likely part of the floor plane
    non_floor_indices = points[:, 2] < (lowest_height - floor_height_threshold)
    non_floor_points = points[non_floor_indices]

    if len(non_floor_points) == 0:
        return np.zeros((0, 3), dtype=np.float64)

    ## remove noise
    non_floor_points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(non_floor_points))

    non_floor_points, _ = non_floor_points.remove_radius_outlier(nb_points=2, radius=0.1)
    non_floor_points, _ = non_floor_points.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    return np.asarray(non_floor_points.points)


## [Filter]
def downsample_points(pcd, voxel_size=0.02):
    """ Downsample points using voxel downsampling.

    Args:
        pcd: (open3d.geometry.PointCloud).
        voxel_size (float): Size of the voxel for downsampling.
    """
    if pcd is None or len(pcd.points) == 0:
        return pcd
    return pcd.voxel_down_sample(voxel_size=voxel_size)


## [Compute]
def compute_metrics_with_grid(all_collections_points, floor_height, grid_size=0.1, alert_height=0.0, area_scale=1.0,
                              height_scale=1.0,
                              min_valid_collections=2):
    """
    Args:
        all_collections_points (np.ndarray): Store collection_times_per_cycle batches of goods in a flat point cloud.
        floor_height (float): Height of the floor plane.
        grid_size_cm (float): grid side length, in cm
        alert_height (float): Alarm height (m), used to determine the quadrant
        area_scale (float): Scale factor for the computed area.
        height_scale (float): Scale factor for the computed height.
    """
    import numpy as np
    if not all_collections_points:
        return 0.0, 0.0, 0.0, 0.0, 0  # volume, area, max_h, mean_h, quadrant

    # Combine all non-empty collections to determine overall spatial extent
    all_points = np.vstack([p for p in all_collections_points if len(p) > 0])
    if len(all_points) == 0:
        return 0.0, 0.0, 0.0, 0.0, 0

    # Calculate XY boundaries of the combined point cloud
    min_x, max_x = np.min(all_points[:, 0]), np.max(all_points[:, 0])
    min_y, max_y = np.min(all_points[:, 1]), np.max(all_points[:, 1])

    # Determine grid dimensions based on spatial extent and grid size
    grid_x_count = int(np.ceil((max_x - min_x) / grid_size))
    grid_y_count = int(np.ceil((max_y - min_y) / grid_size))

    collection_grid_data = []  # Store grid data for each collection

    for collection_points in all_collections_points:
        if len(collection_points) == 0:
            # Empty collection: store empty dict
            collection_grid_data.append({})
            continue

        # Map each point to its corresponding grid cell
        ix = ((collection_points[:, 0] - min_x) / grid_size).astype(int)
        iy = ((collection_points[:, 1] - min_y) / grid_size).astype(int)

        # Filter out points that fall outside the grid boundaries
        valid_mask = (ix >= 0) & (ix < grid_x_count) & (iy >= 0) & (iy < grid_y_count)
        ix_valid = ix[valid_mask]
        iy_valid = iy[valid_mask]
        valid_points = collection_points[valid_mask]

        # Create grid dictionary: key=(gx, gy), value=point indices
        grid_dict = {}
        for idx, (gx, gy) in enumerate(zip(ix_valid, iy_valid)):
            grid_dict.setdefault((gx, gy), []).append(idx)

        # For each grid, store the maximum height in this collection
        grid_heights = {}
        for (gx, gy), idx_list in grid_dict.items():
            cell_points = valid_points[idx_list]
            # Calculate height for each point and take maximum
            heights = floor_height - cell_points[:, 2]
            valid_heights = heights[heights > 0]  # Only consider points above floor
            if len(valid_heights) > 0:
                grid_heights[(gx, gy)] = np.max(valid_heights)

        collection_grid_data.append(grid_heights)

    # Count the effective collection times and total height of each grid
    grid_valid_counts = np.zeros((grid_x_count, grid_y_count), dtype=int)
    grid_height_sums = np.zeros((grid_x_count, grid_y_count))

    # Process each collection's grid data
    for grid_heights in collection_grid_data:
        for (gx, gy), height in grid_heights.items():
            if 0 <= gx < grid_x_count and 0 <= gy < grid_y_count:
                grid_valid_counts[gx, gy] += 1
                grid_height_sums[gx, gy] += height

    total_volume = 0.0
    total_area = 0.0
    heights = []
    max_height = -1
    max_height_xy = (0, 0)
    cell_area = grid_size * grid_size

    # Process each grid cell to compute final metrics
    for gx in range(grid_x_count):
        for gy in range(grid_y_count):
            n = grid_valid_counts[gx, gy]  # Number of collections that detected this grid

            if n >= min_valid_collections:
                # Height = average of the highest points in n collections
                avg_height = (grid_height_sums[gx, gy] / n) * height_scale
                volume = avg_height * (cell_area * area_scale)
                area = cell_area * area_scale

                total_volume += volume
                total_area += area
                heights.append(avg_height)

                if avg_height > max_height:
                    max_height = avg_height
                    max_height_xy = (min_x + (gx + 0.5) * grid_size, min_y + (gy + 0.5) * grid_size)

    mean_height = np.mean(heights) if len(heights) > 0 else 0.0

    quadrant = 0
    if max_height > alert_height:
        x, y = max_height_xy
        if x >= 0 and y >= 0:
            quadrant = 1
        elif x < 0 and y >= 0:
            quadrant = 2
        elif x < 0 and y < 0:
            quadrant = 3
        elif x >= 0 and y < 0:
            quadrant = 4

    return total_volume, total_area, max_height, mean_height, quadrant


## [Rendering]
def render_scene(points, width=720, height=540):
    """ Render the point cloud using Open3D's offscreen renderer.

    Args:
        points (np.ndarray): Array of points with shape (N, 3).
        width (int): Width of the rendered image.
        height (int): Height of the rendered image.
    """
    z = points[:, 2]
    norm_z = (z - np.min(z)) / (np.max(z) - np.min(z) + 1e-8)
    norm_z = (norm_z * 255).astype(np.uint8)
    colors = COLORS_MAP[norm_z] / 255.0  # Normalize colors to [0, 1]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    ## create an offscreen renderer
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=width, height=height)
    vis.add_geometry(pcd)

    ## set up the camera
    ctr = vis.get_view_control()
    ctr.set_lookat([0, 0, 0])
    ctr.set_front([0, 0, -10])
    ctr.set_up([1, 0, 0])

    ## render the scene
    vis.poll_events()
    vis.update_renderer()
    img = np.asarray(vis.capture_screen_float_buffer(do_render=True))
    vis.destroy_window()

    return (img * 255).astype(np.uint8)


## [Http]
HOST_API = '/zlkj-government-boot'
AUTHORIZATION = '/authentication/form'
DATA_UPLOAD = '/zagy-sptsjzx/sptsjzx/yhbz/qyxx/gdssbj'
FILE_UPLOAD = '/file/uploadFileToSj'


## [Http]
def authorization(args):
    url = "http://{}:{}{}{}".format(
        args.server_address, args.server_port,
        HOST_API, AUTHORIZATION
    )
    params = {
        'username': args.username,
        'password': args.password,
    }

    try:
        return requests.post(url, params=params, timeout=5)
    except requests.exceptions.Timeout:
        logger.error("Authentication request timeout: the server did not respond within 5 seconds")
        return None
    except requests.exceptions.RequestException as e:
        logger.error(f"Certified network request error：{e}")
        return None


## [Http]
def upload_data(args, data):
    url = "http://{}:{}{}{}".format(
        args.server_address, args.server_port,
        HOST_API, DATA_UPLOAD
    )

    headers = {
        'Content-Type': 'application/json;charset=utf8',
        'Token': args.tokens
    }

    upload_data_json = [{**data}]

    try:
        response = requests.post(url, json=upload_data_json, headers=headers, timeout=5)
        return response
    except requests.exceptions.Timeout:
        logger.error("Data upload request timeout: the server did not respond within 5 seconds.")
        return None
    except requests.exceptions.RequestException as e:
        logger.error(f"Data upload network request error：{e}")
        return None


## [Http]
def upload_file(args, file_path):
    url = "http://{}:{}{}{}".format(
        args.f_server_address, args.f_server_port,
        HOST_API, FILE_UPLOAD
    )

    headers = {
        'companyCode': args.company_id,
        # 'Token': args.tokens
    }

    if file_path.endswith('.pcd'):
        mime_type = 'application/octet-stream'
        read_mode = 'rb'
    elif file_path.endswith('.png') or file_path.endswith('.jpg') or file_path.endswith('.jpeg'):
        mime_type = 'image/png'
        read_mode = 'rb'
    elif file_path.endswith('.txt') or file_path.endswith('.log'):
        mime_type = 'text/plain'
        read_mode = 'r'
    else:
        raise ValueError("Unsupported file type for upload. Supported types are .pcd, .png, .jpg, .jpeg, .txt, .log")

    try:
        with open(file_path, read_mode) as f:
            files = {
                'file': (file_path.split('/')[-1], f, mime_type)
            }
            response = requests.post(url, files=files, headers=headers, timeout=5)
            return response
    except requests.exceptions.Timeout:
        logger.error("File upload request timeout: the server did not respond within 5 seconds")
        return None
    except requests.exceptions.RequestException as e:
        logger.error(f"File upload network request error：{e}")
        return None
    except Exception as e:
        logger.error(f"File upload processing error：{e}")
        return None


## [Http]
def authenticate_with_reporting_server(args):
    """ Authenticate with the reporting server.

    Args:
        args: Command line arguments containing server address, port, username, and password.
    """
    try:
        response = authorization(args)
        if response is None:
            return ''
    except Exception as e:
        logger.warning(f"Authentication failed with exception: {e}")
        return ''

    if response.status_code == 200:
        tokens = response.json().get('token', '')
        if tokens:
            logger.info("Authentication successful.")
        else:
            logger.warning(f"Authentication failed with message: {response.text}.")
        return tokens
    else:
        logger.warning(f"Authentication failed with status code: {response.status_code}.")
        return ''


## [Http]
def upload_data_to_reporting_server(args, data):
    """ Upload data to the reporting server.

    Args:
        args: Command line arguments containing server address, port, and authentication token.
        data: Data to be uploaded.
    """
    if args.tokens == '':
        args.tokens = authenticate_with_reporting_server(args)
        if args.tokens == '':
            return False

    try:
        response = upload_data(args, data)
        if response is None:
            return False
    except Exception as e:
        logger.error(f"Failed to upload data with exception: {e}")
        return False
    response_json = response.json()

    if response.status_code == 200 and response_json.get('state') == 1:
        logger.info("Data uploaded successfully.")
        return True

    if (response.status_code == 200 and response_json.get('state') == 0) or \
            (response.status_code == 401 and response_json.get('state') == 0):
        ## just try once to re-authenticate
        logger.info("Token expired or invalid, re-authenticating...")
        args.tokens = authenticate_with_reporting_server(args)

        if args.tokens == '':
            logger.warning("Re-authentication failed.")
            return False

        try:
            response = upload_data(args, data)
            if response is None:
                return False
        except Exception as e:
            logger.error(f"Failed to upload data after re-authentication with exception: {e}")
            return False
        response_json = response.json()

        if response.status_code == 200 and response_json.get('state') == 1:
            logger.info("Data uploaded successfully after re-authentication.")
            return True
        else:
            logger.warning(
                f"Failed to upload data after re-authentication with status code: {response.status_code}, message: {response.text}.")
            return False

    logger.warning(
        f"Failed to upload data. Unexpected response with status code: {response.status_code}, message: {response.text}.")
    return False


## [Http]
def upload_file_to_reporting_server(args, file_path):
    """ Upload a file to the reporting server.

    Args:
        args: Command line arguments containing server address, port, and authentication token.
        file_path: Path to the file to be uploaded.
    """
    # if args.tokens == '':
    #     args.tokens = authenticate_with_reporting_server(args)
    #     if args.tokens == '':
    #         return False, ''

    try:
        response = upload_file(args, file_path)
        if response is None:
            return False, ''
    except Exception as e:
        logger.error(f"Failed to upload file with exception: {e}")
        return False, ''
    response_json = response.json()

    if response.status_code == 200 and response_json.get('success'):
        logger.info("File uploaded successfully.")
        return True, response_json.get('result', {}).get('filePath', '')

    logger.warning(
        f"Failed to upload file. Unexpected response with status code: {response.status_code}, message: {response.text}.")
    return False, ''