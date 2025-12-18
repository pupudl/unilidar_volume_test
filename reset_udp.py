import os, sys, time
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build'))
import lidar

manager = lidar.LidarManager()
manager.initLidarWithUDP(
    "192.168.1.62", 6101, # lidar IP and port
    "192.168.1.2", 6201 # local IP and port
)
time.sleep(3)

manager.startLidar()
time.sleep(20)

manager.setWorkMode(8)
time.sleep(1)

manager.resetLidar()
time.sleep(1)