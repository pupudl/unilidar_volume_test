#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <iomanip>

#include "unitree_lidar_sdk.h"
using namespace unilidar_sdk2;

typedef std::tuple<float, float, float, float, float, uint32_t> _point_t;

void hello() {
    std::cout << "Hello world!" << std::endl;
}

class LidarManager {
public:
    LidarManager() {
        std::cout << "[System] LidarManager created!" << std::endl;
    }
    ~LidarManager() {
        std::cout << "[System] LidarManager destroyed!" << std::endl;
    }

    UnitreeLidarReader *lreader;

    void initLidarWithUDP(const std::string &lidar_ip, unsigned short lidar_port,
                          const std::string &local_ip, unsigned short local_port) {
        lreader = createUnitreeLidarReader();

        std::cout << "[System] Initializing Lidar in UDP mode..." << std::endl;

        if (lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip)) {
            std::cout << "[System] Unilidar initialization failed! Exit here!" << std::endl;
            exit(-1);
        } else {
            std::cout << "[System] Unilidar initialization succeed!" << std::endl;
        }
    }

    void initLidarWithSerial() {
        lreader = createUnitreeLidarReader();

        std::cout << "[System] Initializing Lidar in Serial mode..." << std::endl;

        std::string port = "/dev/ttyACM0";
        uint32_t baudrate = 4000000;

        if (lreader->initializeSerial(port, baudrate)) {
            std::cout << "[System] Unilidar initialization failed! Exit here!" << std::endl;
            exit(-1);
        } else {
            std::cout << "[System] Unilidar initialization succeed!" << std::endl;
        }
    }

    void stopLidar() {
        lreader->stopLidarRotation();
        std::cout << "[System] Lidar stopped!" << std::endl;
        sleep(3);
    }

    void startLidar() {
        lreader->startLidarRotation();
        std::cout << "[System] Lidar started!" << std::endl;
        sleep(3);
    }

    void resetLidar() {
        lreader->resetLidar();
        std::cout << "[System] Lidar reset!" << std::endl;
        sleep(3);
    }

    void setWorkMode(int mode) {
        // uint32_t workMode = 0; // 0b0000, open imu, udp
        // uint32_t workMode = 4; // 0b0100, close imu, udp
        // uint32_t workMode = 8; // 0b1000, open imu, serial
        // uint32_t workMode = 12; // 0b1100, close imu, serial
        // uint32_t workMode = 28; // 0b11100
        uint32_t workMode = mode; // Use the provided mode
        lreader->setLidarWorkMode(workMode);
        std::cout << "[System] Lidar work mode set to: " << workMode << std::endl;
        sleep(3);
    }

    void setLidarIPPort(const uint8_t lidar_ip_byte_0, 
                        const uint8_t lidar_ip_byte_1,
                        const uint8_t lidar_ip_byte_2,
                        const uint8_t lidar_ip_byte_3,
                        unsigned short lidar_port, 
                        const uint8_t user_ip_byte_0,
                        const uint8_t user_ip_byte_1,
                        const uint8_t user_ip_byte_2,
                        const uint8_t user_ip_byte_3,
                        unsigned short user_port,
                        const uint8_t gateway_byte_0,
                        const uint8_t gateway_byte_1,
                        const uint8_t gateway_byte_2,
                        const uint8_t gateway_byte_3) {
        
        // Set lidar ip address
        LidarIpAddressConfig config;

        config.lidar_ip[0] = lidar_ip_byte_0;
        config.lidar_ip[1] = lidar_ip_byte_1;
        config.lidar_ip[2] = lidar_ip_byte_2;
        config.lidar_ip[3] = lidar_ip_byte_3;

        config.user_ip[0] = user_ip_byte_0;
        config.user_ip[1] = user_ip_byte_1;
        config.user_ip[2] = user_ip_byte_2;
        config.user_ip[3] = user_ip_byte_3;

        config.lidar_port = lidar_port;
        config.user_port = user_port;

        // Set gateway and subnet mask to default values
        config.gateway[0] = gateway_byte_0;
        config.gateway[1] = gateway_byte_1;
        config.gateway[2] = gateway_byte_2;
        config.gateway[3] = gateway_byte_3;

        config.subnet_mask[0] = 255;
        config.subnet_mask[1] = 255;
        config.subnet_mask[2] = 255;
        config.subnet_mask[3] = 0;

        lreader->setLidarIpAddressConfig(config);
        std::cout << "[System] Lidar IP address is reset! Please reboot the Lidar!" << std::endl;

        sleep(3);
    }

    void setLidarMac(const uint8_t mac_byte_0, 
                     const uint8_t mac_byte_1,
                     const uint8_t mac_byte_2,
                     const uint8_t mac_byte_3,
                     const uint8_t mac_byte_4,
                     const uint8_t mac_byte_5) {
        
        // Set lidar mac address
        LidarMacAddressConfig config;
        config.mac[0] = mac_byte_0;
        config.mac[1] = mac_byte_1;
        config.mac[2] = mac_byte_2;
        config.mac[3] = mac_byte_3;
        config.mac[4] = mac_byte_4;
        config.mac[5] = mac_byte_5;

        config.reserve[0] = 0;
        config.reserve[1] = 0;

        lreader->setLidarMacAddressConfig(config);
        std::cout << "[System] Lidar Mac address is reset! Please reboot the Lidar!" << std::endl;

        sleep(3);
    }

    void workInLoop() {
        int result;

        while (true) {
            result = lreader->runParse();
            
            switch (result) {
                case LIDAR_ACK_DATA_PACKET_TYPE:
                    // Handle ACK data packet
                    std::cout << "Lidar ACK data packet type: " << result << std::endl;
                    break;
                    
                case LIDAR_POINT_DATA_PACKET_TYPE:
                    // Handle point cloud data
                    std::cout << "Lidar point data packet type: " << result << std::endl;
                    break;
                
                case LIDAR_2D_POINT_DATA_PACKET_TYPE:
                    // Handle 2D point cloud data
                    std::cout << "Lidar 2D point data packet type: " << result << std::endl;
                    break;

                case LIDAR_IMU_DATA_PACKET_TYPE:
                    // Handle IMU data
                    std::cout << "Lidar IMU data packet type: " << result << std::endl;
                    break;
                
                case LIDAR_VERSION_PACKET_TYPE:
                    // Handle version data
                    std::cout << "Lidar version packet type: " << result << std::endl;
                    break;
                    
                default:
                    // Handle other packet types or no valid message
                    std::cout << "No valid message parsed or unhandled packet type: " << result << std::endl;
                    break;
            }
        }
    }

    void getVersion() {
        std::string versionSDK;
        std::string versionHardware;
        std::string versionFirmware;

        while (!lreader->getVersionOfLidarFirmware(versionFirmware)) {
            lreader->runParse();
        }
        lreader->getVersionOfLidarHardware(versionHardware);
        lreader->getVersionOfSDK(versionSDK);

        std::cout << "[Data] Lidar hardware version = " << versionHardware << std::endl
                  << "[Data] Lidar firmware version = " << versionFirmware << std::endl
                  << "[Data] Lidar SDK version = " << versionSDK << std::endl;
        sleep(1);
    }

    void getDirtyPercentage() {
        float dirtyPercentage;
        while (!lreader->getDirtyPercentage(dirtyPercentage)) {
            lreader->runParse();
        }
        std::cout << "[Data] Dirty percentage = " << dirtyPercentage << " %" << std::endl;
        sleep(1);
    }

    void getTimeDelay() {
        double timeDelay;
        while (!lreader->getTimeDelay(timeDelay)) {
            lreader->runParse();
        }
        std::cout << "[Data] Time delay (second) = " << timeDelay << std::endl;
        sleep(1);
    }

    std::vector<_point_t> getPointCloudBatch(int batchNum) {
        int result;
        PointCloudUnitree cloud;

        int count = 0;
        std::vector<_point_t> points;

        while (true) {
            result = lreader->runParse();

            if (result == LIDAR_POINT_DATA_PACKET_TYPE) {
                if (lreader->getPointCloud(cloud)) {
                    // double stamp = cloud.stamp;
                    // uint32_t id = cloud.id;
                    // uint32_t ringNum = cloud.ringNum;
                    // std::vector<PointUnitree> points = cloud.points;

                    // std::cout << std::fixed << std::setprecision(6);
                    // std::cout << "[Data] Point cloud parsed with stamp: " << stamp
                    //           << ", id: " << id
                    //           << ", ringNum: " << ringNum
                    //           << ", number of points: " << points.size() << std::endl;

                    // // print first 5 points
                    // for (int i = 0; i < std::min(5, static_cast<int>(points.size())); i++) {
                    //     const PointUnitree &point = points[i];
                    //     std::cout << "[Point " << i << "] x: " << point.x
                    //               << ", y: " << point.y
                    //               << ", z: " << point.z
                    //               << ", intensity: " << point.intensity
                    //               << ", time: " << point.time
                    //               << ", ring: " << point.ring << std::endl;
                    // }
                    for (const auto &point : cloud.points) {
                        // Create a tuple for each point
                        _point_t p = std::make_tuple(point.x, point.y, point.z, point.intensity, point.time, point.ring);
                        points.push_back(p);
                    }
                    
                    // std::cout << "[Data] Processed " << count << " batches of point cloud data." << std::endl;
                    // std::cout << "[Data] Current batch size: " << points.size() << std::endl;
                    
                    count++;
                }
            }

            if (count >= batchNum) break;
        }

        return points;
    }
};


PYBIND11_MODULE(lidar, m) {
    m.doc() = "Pybind11 module for Unitree Lidar SDK";
    m.def("hello", &hello, "Hello world from Unitree Lidar SDK!!!");

    pybind11::class_<LidarManager>(m, "LidarManager")
        .def(pybind11::init<>())
        .def("initLidarWithUDP", &LidarManager::initLidarWithUDP, "Initialize the Lidar with UDP")
        .def("initLidarWithSerial", &LidarManager::initLidarWithSerial, "Initialize the Lidar with Serial")
        .def("stopLidar", &LidarManager::stopLidar, "Stop the Lidar rotation")
        .def("startLidar", &LidarManager::startLidar, "Start the Lidar rotation")
        .def("resetLidar", &LidarManager::resetLidar, "Reset the Lidar")
        .def("setWorkMode", &LidarManager::setWorkMode, "Set the Lidar work mode")
        .def("setLidarIPPort", &LidarManager::setLidarIPPort, "Set the Lidar IP address and port")
        .def("setLidarMac", &LidarManager::setLidarMac, "Set the Lidar MAC address")

        .def("getVersion", &LidarManager::getVersion, "Get the Lidar version")
        .def("getDirtyPercentage", &LidarManager::getDirtyPercentage, "Get the dirty percentage of the Lidar")
        .def("getTimeDelay", &LidarManager::getTimeDelay, "Get the time delay of the Lidar")
        .def("getPointCloudBatch", &LidarManager::getPointCloudBatch, "Get point cloud data in batch")

        .def("workInLoop", &LidarManager::workInLoop, "Process Lidar data");
}