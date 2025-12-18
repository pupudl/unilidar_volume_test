## write a socket server to receive volume data from the lidar manager
import sys
import time
import socket
import struct
import datetime
import numpy as np
import open3d as o3d
from multiprocessing import Process, Queue, Event

from pylib.args import get_server_parser, server_gui_args
from pylib.misc import COLORS_MAP

class PointCloudVisualizer:
    def __init__(self, q):
        self.q = q

        self.app = o3d.visualization.gui.Application.instance
        self.app.initialize()
        font_sans_serif = o3d.visualization.gui.FontDescription(
            typeface="sans-serif", style=o3d.visualization.gui.FontStyle.NORMAL, point_size=22)
        font_id_sans_serif = self.app.add_font(font_sans_serif)

        self._running = True

        self.infor_format = "Volume: {:.6f} m³\nArea: {:.6f} m²\nMax height: {:.2f} m\nMean height: {:.2f} m\nLowest_z: {:.2f} m"
        self.infor = self.infor_format.format(0, 0, 0, 0, 0)

        ## create window for point cloud visualization
        self.window = self.app.create_window("Point Cloud Visualizer", 1024, 768)

        ## add a scene widget to the window
        self.scene = o3d.visualization.gui.SceneWidget()
        self.scene.scene = o3d.visualization.rendering.Open3DScene(self.window.renderer)
        self.scene.scene.set_background([0, 0, 0, 1])
        self.window.add_child(self.scene)

        ## add a label to the window for displaying information
        self.label_width = 220
        self.label_height = 115
        self.label = o3d.visualization.gui.Label(self.infor)
        self.label.font_id = font_id_sans_serif
        self.label.text_color = o3d.visualization.gui.Color(0, 0.9, 0.1)
        self.window.add_child(self.label)

        ## set camera
        bounds = o3d.geometry.AxisAlignedBoundingBox([-2, -2, -2], [2, 2, 2])
        self.scene.setup_camera(60, bounds, bounds.get_center())
        self.scene.look_at(
            [0, 0, 1],
            [-5, -5, -5],
            [0, 0, -1]
        )

        ## add a axis to the scene
        self.axis_material = o3d.visualization.rendering.MaterialRecord()
        self.axis_material.shader = "defaultLit"  # 支持颜色和光照
        self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        self.scene.scene.add_geometry("axis", self.axis, material=self.axis_material)
        
        ## initialize point cloud
        self.material = o3d.visualization.rendering.MaterialRecord()
        self.material.shader = "defaultUnlit"
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3), dtype=np.float32))
        self.scene.scene.add_geometry("pcd", self.pcd, self.material)

        ## set the layout and tick event handlers
        self.window.set_on_layout(self.on_layout)
        self.window.set_on_tick_event(self.on_tick)

    def run(self):
        self.app.run()

    def get_colors(self, z, intensity):
        N, D = intensity.shape

        if D == 1:
            if intensity.sum() == 0:
                # no intensity data, use z for coloring
                norm_z = (z - np.min(z)) / (np.max(z) - np.min(z) + 1e-8)
                norm_z = (norm_z * 191).astype(np.uint8) # using 192/256 colors from the COLORS_MAP
                colors = COLORS_MAP[norm_z] / 255.0
            else:
                # use intensity for coloring
                intensity = intensity.reshape(-1) # intensity ranges from 0 to 255
                norm_intensity = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity) + 1e-8)
                norm_intensity = (norm_intensity * 191).astype(np.uint8) # using 192/256 colors from the COLORS_MAP
                colors = COLORS_MAP[norm_intensity] / 255.0
        elif D == 3:
            colors = intensity
        else:
            colors = np.zeros((N, 3), dtype=np.float32)
        
        return colors

    def on_layout(self, layout_context):
        r = self.window.content_rect
        self.scene.frame = r
        self.label.frame = o3d.visualization.gui.Rect(r.x + 10, r.y + 10, self.label_width, self.label_height)

    def on_tick(self):
        if not self._running:
            self.app.quit()
        else:
            try:
                if not self.q.empty():
                    volume, area, max_height, mean_height, lowest_z, xyz, intensity = self.q.get(block=False)
                    col = self.get_colors(xyz[:, 2], intensity)
                    self.pcd.points = o3d.utility.Vector3dVector(xyz)
                    self.pcd.colors = o3d.utility.Vector3dVector(col)

                    self.infor = self.infor_format.format(
                        volume, area, max_height, mean_height, lowest_z
                    )

                    self.scene.scene.remove_geometry("pcd")
                    self.scene.scene.add_geometry("pcd", self.pcd, self.material)

                    self.label.text = self.infor
                    return True
            except Exception as e:
                print(f"Error in on_tick: {e}")

## Data collection process to receive data through a socket
def data_collection_process(q, stop_event, args):

    def recv_exact(sock, n):
        buf = b''
        while len(buf) < n:
            part = sock.recv(n - len(buf))
            if not part:
                raise ConnectionError("Socket connection closed")
            buf += part
        return buf
    
    HOST = args.host
    PORT = args.port

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        s.settimeout(2)  # Set a timeout for accepting connections
        print(f"Server listening on {HOST}:{PORT}...\n")

        while not stop_event.is_set():
            try:
                conn, addr = s.accept()
                with conn:
                    meta_len = struct.unpack('>I', recv_exact(conn, 4))[0]
                    meta = recv_exact(conn, meta_len)
                    volume, area, max_height, mean_height, lowest_z = struct.unpack('>fffff', meta[:20])
                    data_shape = struct.unpack('>II', meta[20:28])

                    data_len = struct.unpack('>I', recv_exact(conn, 4))[0]
                    data_bytes = recv_exact(conn, data_len)
                    points = np.frombuffer(data_bytes, dtype=np.float32).reshape(data_shape)

                    log_str = "[{}] Received {} points from {}: volume {:.6f} m^3, area {:.6f} m^2, max_height {:.2f} m, mean_height {:.2f} m, lowest_z {:.2f} m.".format(
                        datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                        data_shape, addr,
                        volume, area, max_height, mean_height, lowest_z
                    )
                    print(log_str)

                    ## send points to the visualizer process
                    if points.shape[1] == 3:
                        xyz = points
                        intensity = np.zeros((points.shape[0], 1), dtype=np.float32)
                    elif points.shape[1] == 4:
                        xyz = points[:, :3]
                        intensity = points[:, 3]
                    elif points.shape[1] == 6:
                        xyz = points[:, :3]
                        intensity = points[:, 3:]
                    else:
                        print(f"Received points with unexpected shape: {points.shape}")
                        continue
                    q.put((volume, area, max_height, mean_height, lowest_z, xyz, intensity), block=False)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"An error occurred while processing data: {e}")

    print("Data collection process terminated.")

def run_logic(args):
    q = Queue()
    stop_event = Event()

    col_data_process = Process(target=data_collection_process, args=(q, stop_event, args))
    col_data_process.start()
    time.sleep(1)  # Give some time for the server to start

    try:
        vis = PointCloudVisualizer(q)
        vis.run()
    except Exception as e:
        print(f"An error occurred in the visualizer: {e}")
    finally:
        ## invoke the stop event to terminate the data collection process
        print("Waiting for data collection process to finish...")
        stop_event.set()
        col_data_process.join()
        
    print("Process completed.")

def main():
    parser = get_server_parser()

    ## get pre-args, and set the server GUI args
    pre_args, _ = parser.parse_known_args()
    args = server_gui_args(pre_args)
    
    ## Run the main logic of the application
    run_logic(args)

if __name__ == "__main__":
    if sys.platform.startswith('win'):
        import multiprocessing
        multiprocessing.freeze_support()  # For Windows compatibility with multiprocessing
    main()
