import os, sys
import json
import argparse
import tkinter as tk
import tkinter.font as tkfont

from pylib.misc import generate_stamp

preferred_fonts = ['SimHei', 'Microsoft YaHei', 'Noto Sans CJK SC', 'PingFang SC', 'Songti SC', 'Kaiti SC',
                   'fangsong ti', 'song ti']


def load_config(path: str) -> dict:
    if os.path.exists(path):
        with open(path, 'r') as f:
            return json.load(f)
    return {}


def save_config(path: str, config: dict):
    with open(path, 'w') as f:
        json.dump(config, f, indent=4)


def get_client_parser(DEFAULT_CONFIG_FILE='.config'):
    """ Get the argument parser for the client application.

    Args:
        DEFAULT_CONFIG_FILE (str): Path to the default configuration file in json format.
    """
    ## load default configuration
    defaults = load_config(DEFAULT_CONFIG_FILE)

    ## create the argument parser
    parser = argparse.ArgumentParser(description="Volume computation of point cloud.")

    parser.add_argument('--cli', action='store_true',
                        help="Run in CLI mode instead of GUI. Defaults to GUI mode.")

    parser.add_argument('--save',
                        action='store_true', default=defaults.get('save', False),
                        help="Save the current configuration to the default config path.")
    parser.add_argument('--save_point_cloud',
                        action='store_true', default=defaults.get('save_point_cloud', False),
                        help="Save the Lidar scanned point cloud.")

    ## reporting server
    parser.add_argument('--upload_data',
                        action='store_true', default=defaults.get('upload_data', False),
                        help="Upload the volume data to the reporting server.")
    parser.add_argument('--upload_file',
                        action='store_true', default=defaults.get('upload_file', False),
                        help="Upload the point cloud file to the reporting server.")
    parser.add_argument('--server_address',
                        type=str, default=defaults.get('server_address', ''),
                        help="Address of the server to report the volume.")
    parser.add_argument('--server_port',
                        type=int, default=defaults.get('server_port', -1),
                        help="Port of the server to report the volume.")
    parser.add_argument('--f_server_address',
                        type=str, default=defaults.get('f_server_address', ''),
                        help="Address of the server to report the point cloud file.")
    parser.add_argument('--f_server_port',
                        type=int, default=defaults.get('f_server_port', -1),
                        help="Port of the server to report the point cloud file.")
    parser.add_argument('--username',
                        type=str, default=defaults.get('username', ''),
                        help="Username for server reporting.")
    parser.add_argument('--password',
                        type=str, default=defaults.get('password', ''),
                        help="Password for server reporting.")
    parser.add_argument('--company_name',
                        type=str, default=defaults.get('company_name', 'Test Company'),
                        help="Company name for server reporting.")
    parser.add_argument('--company_id',
                        type=str, default=defaults.get('company_id', '123456'),
                        help="Company ID for server reporting.")
    parser.add_argument('--device_name',
                        type=str, default=defaults.get('device_name', 'Test Device'),
                        help="Device name for server reporting.")
    parser.add_argument('--device_id',
                        type=str, default=defaults.get('device_id', '001'),
                        help="Device ID for server reporting.")
    parser.add_argument('--points_for_uploading',
                        type=int, default=defaults.get('points_for_uploading', 50000),
                        help="Number of points to down-sample for uploading to the server.")
    parser.add_argument('--report_interval',
                        type=int, default=defaults.get('report_interval', 5 * 60),
                        help="Interval in seconds to report the volume to the server.")

    ## visualization server
    parser.add_argument('--visualize',
                        action='store_true', default=defaults.get('visualize', False),
                        help="Visualize the point cloud on the server.")
    parser.add_argument('--v_server_address',
                        type=str, default=defaults.get('v_server_address', '127.0.0.1'),
                        help="Address of the server for visualization.")
    parser.add_argument('--v_server_port',
                        type=int, default=defaults.get('v_server_port', 5001),
                        help="Port of the server for visualization.")

    ## client
    parser.add_argument('--connect_type',
                        type=int, default=defaults.get('connect_type', 0),
                        help="Connection type for the lidar: 0 for UDP, 1 for Serial.")
    parser.add_argument('--update_lowest_height',
                        action='store_true', default=defaults.get('update_lowest_height', True),
                        help="Update the lowest height found in the point cloud.")
    parser.add_argument('--enable_start_stop',
                        action='store_true', default=defaults.get('enable_start_stop', False),
                        help="Enable start/stop functionality for the lidar.")
    parser.add_argument('--point_batch',
                        type=int, default=defaults.get('point_batch', 12),
                        help="Batch number of points to process at once.")
    parser.add_argument('--gather_times',
                        type=int, default=defaults.get('gather_times', 1),
                        help="Number of times to gather point cloud data.")
    parser.add_argument('--lowest_height',
                        type=float, default=defaults.get('lowest_height', -1.0),
                        help="Initial lowest height of the floor in meters.")
    parser.add_argument('--floor_height_threshold',
                        type=float, default=defaults.get('floor_height_threshold', 0.1),
                        help="Threshold for floor height detection in meters.")
    parser.add_argument('--lidar_height_threshold',
                        type=float, default=defaults.get('lidar_height_threshold', 0.5),
                        help="Threshold for lidar height detection in meters.")
    parser.add_argument('--space_region_threshold',
                        type=float, default=defaults.get('space_region_threshold', 1.5),
                        help="Threshold for space region detection in meters.")
    parser.add_argument('--normal_degrees_threshold',
                        type=float, default=defaults.get('normal_degrees_threshold', 10.0),
                        help="Threshold for normal vector angle in degrees to filter plane points.")
    parser.add_argument('--area_scale',
                        type=float, default=defaults.get('area_scale', 1.0),
                        help="Scale factor for the area of the detected plane.")
    parser.add_argument('--height_scale',
                        type=float, default=defaults.get('height_scale', 1.0),
                        help="Scale factor for the height of the detected plane.")
    parser.add_argument('--alert_height',
                        type=float, default=defaults.get('alert_height', 2.55),
                        help="Height threshold to trigger an alert for the detected plane.")

    parser.add_argument('--lidar_ip',
                        type=str, default=defaults.get('lidar_ip', '192.168.1.62'),
                        help="IP address of the Lidar, default is 192.168.1.62")
    parser.add_argument('--lidar_port',
                        type=int, default=defaults.get('lidar_port', 6101),
                        help="Port number of the Lidar, default is 6101")
    parser.add_argument('--local_ip',
                        type=str, default=defaults.get('local_ip', '192.168.1.2'),
                        help="Local IP address to bind, default is 192.168.1.2")
    parser.add_argument('--local_port',
                        type=int, default=defaults.get('local_port', 6201),
                        help="Local port number to bind, default is 6201")
    parser.add_argument('--start_upload_round',
                        type=int, default=defaults.get('start_upload_round', 1),
                        help="Start uploading data to server from the Nth round. ")
    parser.add_argument('--grid_size',
                        type=float, default=defaults.get('grid_size', 0.1),
                        help="Grid size for volume calculation in meters.")
    parser.add_argument('--collection_times_per_cycle',
                        type=int, default=defaults.get('collection_times_per_cycle', 3),
                        help="Collection frequency within one start stop cycle")
    parser.add_argument('--min_valid_collections',
                        type=int, default=defaults.get('min_valid_collections', 2),
                        help="Minimum threshold for the number of valid data in a grid")

    ## other global arguments
    parser.add_argument('--tokens',
                        type=str, default='',
                        help="Tokens for authentication.")
    parser.add_argument('--stamp',
                        type=str, default=generate_stamp(random_sufix_length=6),
                        help="Stamp for the trail name, default is generated based on current time.")
    parser.add_argument('--DEFAULT_CONFIG_FILE',
                        type=str, default=DEFAULT_CONFIG_FILE,
                        help="Constants for the default configuration file path, i.e., '.config'.")
    parser.add_argument('--POINTS_SAVE_FILE',
                        type=str, default='points.pcd',
                        help="Constants for the point cloud save file name, i.e., 'points.pcd'.")
    parser.add_argument('--LOGS_FOLDER',
                        type=str, default='_logs',
                        help="Constants for the log folder path, i.e., '_logs'.")
    parser.add_argument('--PCDS_FOLDER',
                        type=str, default='_pcds',
                        help="Constants for the point cloud folder path, i.e., '_pcds'.")
    parser.add_argument('--START_LIDAR_WAIT_TIME',
                        type=int, default=20,
                        help="Constants for the wait time in seconds for the Lidar to start, i.e., 20 seconds.")
    parser.add_argument('--HISTORY_WINDOW_SIZE',
                        type=int, default=5,
                        help="Constants for the history window size, i.e., 5.")

    return parser


class ClientGUIArgs(tk.Tk):
    def __init__(self, pre_args):
        super().__init__()
        self.pre_args = pre_args

        self.title("客户端")
        self.geometry("720x640")
        self.resizable(False, False)

        ## font and pad configuration
        fonts = list(tkfont.families())
        font = self._get_preferred_font(fonts)
        self.frame_font_kwargs = {"font": (font, 12), "fg": "green"}
        self.inner_frame_font_kwargs = {"font": (font, 12), "fg": "gray"}
        self.widget_font_kwargs = {"font": (font, 12)}
        self.inner_widget_font_kwargs = {"font": (font, 12), "fg": "gray"}

        self.frame_pad_kwargs = {"padx": 8, "pady": 8, "sticky": "nsew"}
        self.inner_frame_pad_kwargs = {"padx": 2, "pady": 2, "sticky": "nsew"}
        self.widget_pad_kwargs = {"padx": 2, "pady": 2}
        self.inner_widget_pad_kwargs = {"padx": 1, "pady": 1}

        ## create widgets
        self.create_widgets(pre_args)

        ## bind the close event
        self.protocol("WM_DELETE_WINDOW", self.close)

    def _get_preferred_font(self, fonts):
        """Get a preferred font from the list of available fonts for Chinese characters."""
        for font in preferred_fonts:
            if font in fonts:
                return font
        return fonts[0] if fonts else 'Times'

    def create_widgets(self, pre_args):
        self.entries = {}

        self.container = tk.Frame(self)
        self.container.pack(expand=True)

        ## main config
        self.create_reporting_server_widgets(pre_args)
        self.create_visualization_server_widgets(pre_args)
        self.create_client_widgets(pre_args)

        ## save frame that contains the save options
        self.save_frame = tk.Frame(self.container)
        self.save_frame.grid(row=2, column=0, columnspan=2, **self.frame_pad_kwargs)
        self.save_container = tk.Frame(self.save_frame)
        self.save_container.pack(expand=True)

        var = tk.BooleanVar(value=pre_args.save)
        save_check = tk.Checkbutton(self.save_container, text="保存配置", variable=var, **self.widget_font_kwargs)
        save_check.var = var
        save_check.grid(row=0, column=0, **self.widget_pad_kwargs, sticky='e')
        self.entries['save'] = save_check

        var = tk.BooleanVar(value=pre_args.save_point_cloud)
        save_pcd_check = tk.Checkbutton(self.save_container, text="保存点云", variable=var, **self.widget_font_kwargs)
        save_pcd_check.var = var
        save_pcd_check.grid(row=0, column=1, **self.widget_pad_kwargs, sticky='e')
        self.entries['save_point_cloud'] = save_pcd_check

        ## submit button
        submit_button = tk.Button(self.container, text="运行", command=self.submit, **self.widget_font_kwargs)
        submit_button.grid(row=3, column=0, columnspan=2, **self.widget_pad_kwargs)

    def create_reporting_server_widgets(self, pre_args):
        """ Create widgets for report server configuration. """
        r_server_frame = tk.LabelFrame(self.container, text="报告服务器配置", **self.frame_font_kwargs)
        r_server_frame.grid(row=0, column=0, **self.frame_pad_kwargs)

        row = 0

        upload_frame = tk.Frame(r_server_frame)
        upload_frame.grid(row=row, column=0, columnspan=2, **self.frame_pad_kwargs)
        var = tk.BooleanVar(value=pre_args.upload_data)
        widget = tk.Checkbutton(upload_frame, text="上传数据", variable=var, **self.widget_font_kwargs)
        widget.var = var
        widget.grid(row=0, column=0, **self.widget_pad_kwargs)
        self.entries['upload_data'] = widget
        var = tk.BooleanVar(value=pre_args.upload_file)
        widget = tk.Checkbutton(r_server_frame, text="上传文件", variable=var, **self.widget_font_kwargs)
        widget.var = var
        widget.grid(row=0, column=1, **self.widget_pad_kwargs)
        self.entries['upload_file'] = widget
        row += 1

        tk.Label(r_server_frame, text="服务器地址:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                     **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.server_address)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['server_address'] = widget
        row += 1

        tk.Label(r_server_frame, text="服务器端口:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                     **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.server_port)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['server_port'] = widget
        row += 1

        tk.Label(r_server_frame, text="文件服务器地址:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                         **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.f_server_address)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['f_server_address'] = widget
        row += 1

        tk.Label(r_server_frame, text="文件服务器端口:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                         **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.f_server_port)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['f_server_port'] = widget
        row += 1

        tk.Label(r_server_frame, text="用户名:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                 **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.username)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['username'] = widget
        row += 1

        tk.Label(r_server_frame, text="密码:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                               **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.password)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['password'] = widget
        row += 1

        tk.Label(r_server_frame, text="公司名称:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                   **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.company_name)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['company_name'] = widget
        row += 1

        tk.Label(r_server_frame, text="公司 ID:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                  **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.company_id)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['company_id'] = widget
        row += 1

        tk.Label(r_server_frame, text="设备名称:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                   **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.device_name)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['device_name'] = widget
        row += 1

        tk.Label(r_server_frame, text="设备 ID:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                  **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.device_id)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['device_id'] = widget
        row += 1

        tk.Label(r_server_frame, text="上传点数:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                   **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.points_for_uploading)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['points_for_uploading'] = widget
        row += 1

        tk.Label(r_server_frame, text="推送间隔(秒):", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                       **self.widget_pad_kwargs)
        widget = tk.Entry(r_server_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.report_interval)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['report_interval'] = widget
        row += 1

    def create_visualization_server_widgets(self, pre_args):
        """ Create widgets for visualization server configuration. """
        visualize_frame = tk.LabelFrame(self.container, text="可视服务器配置", **self.frame_font_kwargs)
        visualize_frame.grid(row=1, column=0, **self.frame_pad_kwargs)

        var = tk.BooleanVar(value=pre_args.visualize)
        widget = tk.Checkbutton(visualize_frame, text="推送至可视服务器", variable=var, **self.widget_font_kwargs)
        widget.var = var
        widget.grid(row=0, column=0, columnspan=2, **self.widget_pad_kwargs)
        self.entries['visualize'] = widget

        tk.Label(visualize_frame, text="服务器地址:", **self.widget_font_kwargs).grid(row=1, column=0,
                                                                                      **self.widget_pad_kwargs)
        widget = tk.Entry(visualize_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.v_server_address)
        widget.grid(row=1, column=1, **self.widget_pad_kwargs)
        self.entries['v_server_address'] = widget

        tk.Label(visualize_frame, text="服务器端口:", **self.widget_font_kwargs).grid(row=2, column=0,
                                                                                      **self.widget_pad_kwargs)
        widget = tk.Entry(visualize_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.v_server_port)
        widget.grid(row=2, column=1, **self.widget_pad_kwargs)
        self.entries['v_server_port'] = widget

    def create_client_widgets(self, pre_args):
        config_frame = tk.LabelFrame(self.container, text="客户端配置", **self.frame_font_kwargs)
        config_frame.grid(row=0, column=1, rowspan=2, **self.frame_pad_kwargs)

        row = 0
        ## a new frame for connection type
        connect_frame = tk.Frame(config_frame)
        connect_frame.grid(row=row, column=0, columnspan=2, **self.frame_pad_kwargs)

        var = tk.IntVar(value=pre_args.connect_type)
        widget = tk.Label(connect_frame, text="连接方式:", **self.widget_font_kwargs)
        widget.var = var
        widget.grid(row=0, column=0, **self.widget_pad_kwargs)

        udp_radiobutton = tk.Radiobutton(connect_frame, text="网络 UDP", value=0, variable=var,
                                         **self.widget_font_kwargs)
        udp_radiobutton.grid(row=0, column=1, **self.widget_pad_kwargs)
        serial_radiobutton = tk.Radiobutton(connect_frame, text="串口 Serial", value=1, variable=var,
                                            **self.widget_font_kwargs)
        serial_radiobutton.grid(row=0, column=2, **self.widget_pad_kwargs)
        self.entries['connect_type'] = widget
        row += 1

        var = tk.BooleanVar(value=pre_args.update_lowest_height)
        widget = tk.Checkbutton(config_frame, text="更新最低高度", variable=var, **self.widget_font_kwargs)
        widget.var = var
        widget.grid(row=row, column=0, columnspan=2, **self.widget_pad_kwargs)
        self.entries['update_lowest_height'] = widget
        row += 1

        var = tk.BooleanVar(value=pre_args.enable_start_stop)
        widget = tk.Checkbutton(config_frame, text="使用启停功能", variable=var, **self.widget_font_kwargs)
        widget.var = var
        widget.grid(row=row, column=0, columnspan=2, **self.widget_pad_kwargs)
        self.entries['enable_start_stop'] = widget
        row += 1

        tk.Label(config_frame, text="点云批数:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                 **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.point_batch)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['point_batch'] = widget
        row += 1

        tk.Label(config_frame, text="采集次数:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                 **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.gather_times)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['gather_times'] = widget
        row += 1

        tk.Label(config_frame, text="最低高度(m):", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                    **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.lowest_height)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['lowest_height'] = widget
        row += 1

        tk.Label(config_frame, text="地板高度阈值(m):", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                        **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.floor_height_threshold)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['floor_height_threshold'] = widget
        row += 1

        tk.Label(config_frame, text="雷达高度阈值(m):", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                        **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.lidar_height_threshold)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['lidar_height_threshold'] = widget
        row += 1

        tk.Label(config_frame, text="采集区域长度(m):", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                        **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.space_region_threshold)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['space_region_threshold'] = widget
        row += 1

        tk.Label(config_frame, text="法线角度阈值(度):", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                         **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.normal_degrees_threshold)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['normal_degrees_threshold'] = widget
        row += 1

        tk.Label(config_frame, text="面积系数:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                 **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.area_scale)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['area_scale'] = widget
        row += 1

        tk.Label(config_frame, text="高度系数:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                 **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.height_scale)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['height_scale'] = widget
        row += 1

        tk.Label(config_frame, text="警报高度(m):", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                    **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.alert_height)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['alert_height'] = widget
        row += 1

        # 新增：从第N轮开始上传的输入框
        tk.Label(config_frame, text="开始上传轮次:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                     **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.start_upload_round)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['start_upload_round'] = widget
        row += 1

        tk.Label(config_frame, text="体积网格大小(m):", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                        **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.grid_size)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['grid_size'] = widget
        row += 1

        tk.Label(config_frame, text="一次启停内采集数:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                         **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.collection_times_per_cycle)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['collection_times_per_cycle'] = widget
        row += 1

        tk.Label(config_frame, text="最小出现次数阈值:", **self.widget_font_kwargs).grid(row=row, column=0,
                                                                                         **self.widget_pad_kwargs)
        widget = tk.Entry(config_frame, **self.widget_font_kwargs)
        widget.insert(0, pre_args.min_valid_collections)
        widget.grid(row=row, column=1, **self.widget_pad_kwargs)
        self.entries['min_valid_collections'] = widget
        row += 1

        udp_config_frame = tk.LabelFrame(config_frame, text="UDP 配置 (仅 UDP 连接方式使用)",
                                         **self.inner_frame_font_kwargs)
        udp_config_frame.grid(row=row, column=0, columnspan=2, **self.inner_frame_pad_kwargs)
        row += 1

        tk.Label(udp_config_frame, text="雷达地址:", **self.inner_widget_font_kwargs).grid(row=0, column=0,
                                                                                           **self.inner_widget_pad_kwargs)
        widget = tk.Entry(udp_config_frame, **self.inner_widget_font_kwargs, width=15)
        widget.insert(0, pre_args.lidar_ip)
        widget.grid(row=0, column=1, **self.inner_widget_pad_kwargs)
        self.entries['lidar_ip'] = widget
        tk.Label(udp_config_frame, text="端口:", **self.inner_widget_font_kwargs).grid(row=0, column=2,
                                                                                       **self.inner_widget_pad_kwargs)
        widget = tk.Entry(udp_config_frame, **self.inner_widget_font_kwargs, width=5)
        widget.insert(0, pre_args.lidar_port)
        widget.grid(row=0, column=3, **self.inner_widget_pad_kwargs)
        self.entries['lidar_port'] = widget

        tk.Label(udp_config_frame, text="本地地址:", **self.inner_widget_font_kwargs).grid(row=1, column=0,
                                                                                           **self.inner_widget_pad_kwargs)
        widget = tk.Entry(udp_config_frame, **self.inner_widget_font_kwargs, width=15)
        widget.insert(0, pre_args.local_ip)
        widget.grid(row=1, column=1, **self.inner_widget_pad_kwargs)
        self.entries['local_ip'] = widget
        tk.Label(udp_config_frame, text="端口:", **self.inner_widget_font_kwargs).grid(row=1, column=2,
                                                                                       **self.inner_widget_pad_kwargs)
        widget = tk.Entry(udp_config_frame, **self.inner_widget_font_kwargs, width=5)
        widget.insert(0, pre_args.local_port)
        widget.grid(row=1, column=3, **self.inner_widget_pad_kwargs)
        self.entries['local_port'] = widget

    def submit(self):
        for key, entry in self.entries.items():
            if isinstance(entry, tk.Entry):
                value = entry.get()

                origin_value = getattr(self.pre_args, key)
                origin_type = type(origin_value)
                try:
                    setattr(self.pre_args, key, origin_type(value))
                except ValueError:
                    print(f"Could not convert {key}({value}) to {origin_type.__name__}. Please check the input!!!")
                    return  # stop submission if conversion fails
            elif isinstance(entry, tk.Checkbutton):
                var = entry.var
                setattr(self.pre_args, key, bool(var.get()))
            elif isinstance(entry, tk.Label):
                var = entry.var
                setattr(self.pre_args, key,
                        int(var.get()))  # used for a list radio buttons, the var in bind with a Label

        self.destroy()  # Close the GUI after submission

    def close(self):
        self.destroy()
        ## close the whole application
        sys.exit(0)


def client_gui_args(pre_args):
    """ Convert pre_args to GUI arguments.

    Args:
        pre_args (argparse.Namespace): Pre-parsed command line arguments.
    """
    app = ClientGUIArgs(pre_args)
    app.mainloop()

    return pre_args


def get_server_parser():
    """ Get the argument parser for the server application. """
    parser = argparse.ArgumentParser(description="Server for volume computation of point cloud.")

    parser.add_argument('--host', type=str, default='0.0.0.0',
                        help="Host address to bind the server to.")
    parser.add_argument('--port', type=int, default=5001,
                        help="Port to run the server on.")

    return parser


class ServerGUIArgs(tk.Tk):
    def __init__(self, pre_args):
        super().__init__()
        self.pre_args = pre_args

        self.title("服务端")
        self.geometry("360x270")
        self.resizable(False, False)

        fonts = list(tkfont.families())
        self.font = self._get_preferred_font(fonts)
        self.create_widgets(pre_args)

        self.protocol("WM_DELETE_WINDOW", self.close)

    def create_widgets(self, pre_args):
        """ Create the GUI widgets for server configuration. """
        self.entries = {}

        self.container = tk.Frame(self)
        self.container.pack(expand=True)

        row = 0

        tk.Label(self.container, text="地址", font=(self.font, 12)).grid(row=row, column=0, padx=10, pady=10,
                                                                         sticky="e")
        host_entry = tk.Entry(self.container, font=("Arial", 12))
        host_entry.insert(0, pre_args.host)
        host_entry.grid(row=row, column=1, padx=10, pady=10)
        self.entries['host'] = host_entry
        row += 1

        tk.Label(self.container, text="端口", font=(self.font, 12)).grid(row=row, column=0, padx=10, pady=10,
                                                                         sticky="e")
        port_entry = tk.Entry(self.container, font=("Arial", 12))
        port_entry.insert(0, pre_args.port)
        port_entry.grid(row=row, column=1, padx=10, pady=10)
        self.entries['port'] = port_entry
        row += 1

        start_button = tk.Button(self.container, text="开始", font=(self.font, 12), command=self.submit)
        start_button.grid(row=row, columnspan=2, pady=10)

    def _get_preferred_font(self, fonts):
        """Get a preferred font from the list of available fonts for Chinese characters."""
        for font in preferred_fonts:
            if font in fonts:
                return font
        return fonts[0] if fonts else 'Times'

    def submit(self):
        for key, entry in self.entries.items():
            if isinstance(entry, tk.Entry):
                value = entry.get()

                origin_value = getattr(self.pre_args, key)
                origin_type = type(origin_value)
                try:
                    setattr(self.pre_args, key, origin_type(value))
                except ValueError:
                    print(f"Could not convert {key}({value}) to {origin_type.__name__}. Please check the input!!!")
                    return  # stop submission if conversion fails
            elif isinstance(entry, tk.Checkbutton):
                var = entry.var
                setattr(self.pre_args, key, bool(var.get()))

        self.destroy()

    def close(self):
        self.destroy()
        ## close the whole application
        sys.exit(0)


def server_gui_args(pre_args):
    """ Convert pre_args to GUI arguments.

    Args:
        pre_args (argparse.Namespace): Pre-parsed command line arguments.
    """
    app = ServerGUIArgs(pre_args)
    app.mainloop()

    return pre_args