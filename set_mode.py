import time
import argparse

try:
    import lidar
except ImportError:
    try:
        import os, sys
        sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build'))
        import lidar
    except ImportError:
        print("Failed to import the lidar module. Please ensure it is built correctly.")
        sys.exit(1)

def reset_mode(lidar_ip, lidar_port, local_ip, local_port, mode):
    manager = lidar.LidarManager()
    manager.initLidarWithUDP(lidar_ip, lidar_port, local_ip, local_port)

    print("\nTesting connection to Lidar...")
    manager.startLidar()
    time.sleep(20)  # Allow some time for the Lidar to start
    manager.stopLidar()
    time.sleep(2)  # Wait for the Lidar to stop

    print("\nSetting Lidar mode...")
    manager.setWorkMode(mode)
    print("Lidar mode set to {} successfully.\n".format(mode))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Lidar Manager")
    parser.add_argument("--lidar_ip", type=str, default="192.168.1.62",
                        help="IP address of the Lidar, default is 192.168.1.62")
    parser.add_argument("--lidar_port", type=int, default=6101,
                        help="Port number of the Lidar, default is 6101")
    parser.add_argument("--local_ip", type=str, default="192.168.1.2",
                        help="Local IP address to bind, default is 192.168.1.2")
    parser.add_argument("--local_port", type=int, default=6201,
                        help="Local port number to bind, default is 6201")

    parser.add_argument("--mode", type=int, default=0,
                        help="Mode to set for the Lidar, default is 0. 0 [open imu, udp], 4 [close imu, udp], 8 [open imu, serial], 12 [close imu, serial]")
    args = parser.parse_args()

    reset_mode(args.lidar_ip, args.lidar_port, args.local_ip, args.local_port, args.mode)