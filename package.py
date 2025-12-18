import os, sys, platform
from pathlib import Path

## win/linux, x86_64/aarch64
server_install_cmd_fmt = "pyinstaller -y --distpath dist-{sys}-{dev} --workpath pybuild \
--add-binary '{src_o3d}:{dst_o3d}' \
--exclude-module open3d.cuda \
--onedir --console --clean --name server s.py"

## linux, x86_64/aarch64
client_install_cmd_fmt = "pyinstaller -y --distpath dist-{sys}-{dev} --workpath pybuild \
--add-binary 'build/djset.cpython-{py_v_major}{py_v_minor}-{dev}-linux-gnu.so:.' \
--add-binary 'build/lidar.cpython-{py_v_major}{py_v_minor}-{dev}-linux-gnu.so:.' \
--collect-all open3d \
--exclude-module open3d.cuda \
--onedir --console --clean --name client c.py"

## linux, x86_64/aarch64
stop_install_cmd_fmt = "pyinstaller -y --distpath dist-{sys}-{dev} --workpath pybuild \
--add-binary 'build/lidar.cpython-{py_v_major}{py_v_minor}-{dev}-linux-gnu.so:.' \
--onedir --console --clean --name stop stop.py" 

## linux, x86_64/aarch64
start_install_cmd_fmt = "pyinstaller -y --distpath dist-{sys}-{dev} --workpath pybuild \
--add-binary 'build/lidar.cpython-{py_v_major}{py_v_minor}-{dev}-linux-gnu.so:.' \
--onedir --console --clean --name start start.py"

## linux, x86_64/aarch64
set_addr_install_cmd_fmt = "pyinstaller -y --distpath dist-{sys}-{dev} --workpath pybuild \
--add-binary 'build/lidar.cpython-{py_v_major}{py_v_minor}-{dev}-linux-gnu.so:.' \
--onedir --console --clean --name set_addr set_addr.py"

## linux, x86_64/aarch64
set_mode_install_cmd_fmt = "pyinstaller -y --distpath dist-{sys}-{dev} --workpath pybuild \
--add-binary 'build/lidar.cpython-{py_v_major}{py_v_minor}-{dev}-linux-gnu.so:.' \
--onedir --console --clean --name set_mode set_mode.py"

python_exec = sys.executable

system = platform.system().lower()
machine = platform.machine().lower()
python_version = sys.version_info

if system.startswith('win'):
    ## Windows, x86_64
    ## only install server
    enviro_path = Path(python_exec).parent

    src_o3d_resources = Path(enviro_path, 'Lib', 'site-packages', 'open3d', 'resources')
    dst_o3d_resources = Path('open3d', 'resources')
    server_install_cmd = server_install_cmd_fmt.format(
        sys=system, dev=machine, src_o3d=src_o3d_resources, dst_o3d=dst_o3d_resources)
    print(server_install_cmd)
    os.system(server_install_cmd)

if system.startswith('linux'):
    ## Linux, x86_64/aarch64
    ## install all components
    enviro_path = Path(python_exec).parent.parent

    src_o3d_resources = Path(enviro_path, 'lib', 'python{}.{}'.format(python_version.major, python_version.minor), 'site-packages', 'open3d', 'resources')
    if not src_o3d_resources.exists():
        src_o3d_resources = Path('/usr/local/lib/python3.10/dist-packages/open3d/resources')
    dst_o3d_resources = Path('open3d', 'resources')

    server_install_cmd = server_install_cmd_fmt.format(
        sys=system, dev=machine, src_o3d=src_o3d_resources, dst_o3d=dst_o3d_resources)
    client_install_cmd = client_install_cmd_fmt.format(
        sys=system, dev=machine, py_v_major=python_version.major, py_v_minor=python_version.minor)
    stop_install_cmd = stop_install_cmd_fmt.format(
        sys=system, dev=machine, py_v_major=python_version.major, py_v_minor=python_version.minor)
    start_install_cmd = start_install_cmd_fmt.format(
        sys=system, dev=machine, py_v_major=python_version.major, py_v_minor=python_version.minor)
    set_addr_install_cmd_fmt = set_addr_install_cmd_fmt.format(
        sys=system, dev=machine, py_v_major=python_version.major, py_v_minor=python_version.minor)
    set_mode_install_cmd_fmt = set_mode_install_cmd_fmt.format(
        sys=system, dev=machine, py_v_major=python_version.major, py_v_minor=python_version.minor)

    if os.system(server_install_cmd):
        sys.exit(1)
    if os.system(client_install_cmd):
        sys.exit(1)
    if os.system(stop_install_cmd):
        sys.exit(1)
    if os.system(start_install_cmd):
        sys.exit(1)
    if os.system(set_addr_install_cmd_fmt):
        sys.exit(1)
    if os.system(set_mode_install_cmd_fmt):
        sys.exit(1)
