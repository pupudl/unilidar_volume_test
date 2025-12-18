import os, sys, time
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build'))
import lidar

manager = lidar.LidarManager()
manager.initLidarWithSerial()
time.sleep(3)

manager.startLidar()
time.sleep(20)

manager.setWorkMode(0)
time.sleep(1)

manager.resetLidar()
time.sleep(1)


