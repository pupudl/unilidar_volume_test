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

def reset_ip_address(lidar_ip, lidar_port, local_ip, local_port,
                    new_mac,
                    new_lidar_ip, new_lidar_port, new_local_ip, new_local_port, new_gateway):
    manager = lidar.LidarManager()
    manager.initLidarWithUDP(lidar_ip, lidar_port, local_ip, local_port)

    print("\nTesting connection to Lidar...")
    manager.startLidar()
    time.sleep(20)  # Allow some time for the Lidar to start
    manager.stopLidar()
    time.sleep(2)  # Wait for the Lidar to stop

    print("\nResetting Lidar IP and port...")
    try:
        new_mac_bytes = new_mac.split(':')
        new_mac_byte_0, new_mac_byte_1, new_mac_byte_2, new_mac_byte_3, new_mac_byte_4, new_mac_byte_5 = map(lambda x: int(x, 16), new_mac_bytes)
        print("New MAC address: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}".format(
            new_mac_byte_0, new_mac_byte_1, new_mac_byte_2, new_mac_byte_3, new_mac_byte_4, new_mac_byte_5))
        manager.setLidarMac(
            new_mac_byte_0, new_mac_byte_1,
            new_mac_byte_2, new_mac_byte_3,
            new_mac_byte_4, new_mac_byte_5
        )
        print("MAC address reset successfully.\n")

        new_lidar_ip_bytes = new_lidar_ip.split('.')
        new_lidar_ip_byte_0, new_lidar_ip_byte_1, new_lidar_ip_byte_2, new_lidar_ip_byte_3 = map(int, new_lidar_ip_bytes)
        new_lidar_port = int(new_lidar_port)

        new_local_ip_bytes = new_local_ip.split('.')
        new_local_ip_byte_0, new_local_ip_byte_1, new_local_ip_byte_2, new_local_ip_byte_3 = map(int, new_local_ip_bytes)
        new_local_port = int(new_local_port)

        new_gateway_bytes = new_gateway.split('.')
        new_gateway_byte_0, new_gateway_byte_1, new_gateway_byte_2, new_gateway_byte_3 = map(int, new_gateway_bytes)

        print("New Lidar IP and port: {}.{}.{}.{}:{}".format(
            new_lidar_ip_byte_0, new_lidar_ip_byte_1, new_lidar_ip_byte_2, new_lidar_ip_byte_3, new_lidar_port))
        print("New Local IP and port: {}.{}.{}.{}:{}".format(
            new_local_ip_byte_0, new_local_ip_byte_1, new_local_ip_byte_2, new_local_ip_byte_3, new_local_port))
        print("New gateway: {}.{}.{}.{}".format(
            new_gateway_byte_0, new_gateway_byte_1, new_gateway_byte_2, new_gateway_byte_3
        ))

        manager.setLidarIPPort(
            new_lidar_ip_byte_0, new_lidar_ip_byte_1,
            new_lidar_ip_byte_2, new_lidar_ip_byte_3,
            new_lidar_port,
            new_local_ip_byte_0, new_local_ip_byte_1,
            new_local_ip_byte_2, new_local_ip_byte_3,
            new_local_port,
            new_gateway_byte_0, new_gateway_byte_1,
            new_gateway_byte_2, new_gateway_byte_3
        )
        print("Lidar IP and port reset successfully.")
    except Exception as e:
        print(f"Failed to reset Lidar IP and port: {e}")
        sys.exit(1)

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

    parser.add_argument("--random_mac", action='store_true',
                        help="Generate a random MAC address for the Lidar, default is False")
    parser.add_argument("--new_mac", type=str, default="00:00:00:00:00:00",
                        help="New MAC address of the Lidar, default is 00:00:00:00:00:00")

    parser.add_argument("--new_lidar_ip", type=str, default="192.168.1.62",
                        help="New IP address of the Lidar, default is 192.168.1.62")
    parser.add_argument("--new_lidar_port", type=int, default=6101,
                        help="New port number of the Lidar, default is 6101")
    parser.add_argument("--new_local_ip", type=str, default="192.168.1.2",
                        help="New local IP address to bind, default is 192.168.1.2")
    parser.add_argument("--new_local_port", type=int, default=6201,
                        help="New local port number to bind, default is 6201")
    parser.add_argument("--new_gateway", type=str, default="0.0.0.0",
                        help="New Gateway IP, default is 0.0.0.0")
    args = parser.parse_args()

    if args.random_mac:
        import random
        args.new_mac = "02:00:a2:{:02x}:{:02x}:{:02x}".format(
            random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        print(f"Generated random MAC address: {args.new_mac}\n")

    reset_ip_address(
        args.lidar_ip, args.lidar_port, args.local_ip, args.local_port,
        args.new_mac,
        args.new_lidar_ip, args.new_lidar_port, args.new_local_ip, args.new_local_port, args.new_gateway
    )