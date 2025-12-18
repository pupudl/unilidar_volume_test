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

def start(args):
    manager = lidar.LidarManager()
    if args.type == 0:
        manager.initLidarWithUDP(
            args.lidar_ip, args.lidar_port,
            args.local_ip, args.local_port
        )
    elif args.type == 1:
        manager.initLidarWithSerial()
    else:
        print("Unsupported type. Use 0 for UDP and 1 for Serial.")
        return
    
    time.sleep(3)  # Allow some time for the lidar to initialize
    manager.startLidar()
    time.sleep(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Lidar Manager")
    parser.add_argument("-t", "--type", type=int, default=0, help="Work mode: 0 for udp, 1 for serial")
    parser.add_argument("--lidar_ip", type=str, default="192.168.1.62",
                        help="IP address of the Lidar, default is 192.168.1.62")
    parser.add_argument("--lidar_port", type=int, default=6101,
                        help="Port number of the Lidar, default is 6101")
    parser.add_argument("--local_ip", type=str, default="192.168.1.2",
                        help="Local IP address to bind, default is 192.168.1.2")
    parser.add_argument("--local_port", type=int, default=6201,
                        help="Local port number to bind, default is 6201")
    args = parser.parse_args()

    start(args)
