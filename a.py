import os, sys
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build'))
import time
import numpy as np

import lidar

manager = lidar.LidarManager()
manager.initLidarWithUDP(
    "192.168.1.62", 6101, # lidar IP and port
    "192.168.1.2", 6201 # local IP and port
)
# manager.initLidarWithSerial()

while True:
    print("\n")
    print("Starting Lidar...")
    manager.startLidar()
    time.sleep(20)

    # Get a batch of point cloud data
    points = manager.getPointCloudBatch(2)
    points = np.array(points).astype(np.float32)
    xyz = points[:, :3]
    intensity = points[:, 3]
    print(intensity.max(), intensity.min())

    time.sleep(3)
    print("Get ", len(points), " points..")

    manager.stopLidar()
    time.sleep(3)

    print("Stopping Lidar...")

    


