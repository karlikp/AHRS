#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "./../unitree_lidar_sdk/include/unitree_lidar_sdk.h"
#include <csignal>

#include "function.h"

namespace py = pybind11;
using namespace unitree_lidar_sdk;

class UnitreeLidarWrapper {
public:

    UnitreeLidarReader* lreader;
    int cloud_scan_num;
    std::string port_name;
    bool init_ref = false;
    typename PointMatcher<float>::DataPoints ref;
    
    UnitreeLidarWrapper() {
        lreader = createUnitreeLidarReader();
        cloud_scan_num = 18;
        port_name = "/dev/ttyUSB0";
        std::signal(SIGINT, signal_handler); // handling interrupt signal 
    }

    bool initialize() {
        if (lreader->initialize(cloud_scan_num, port_name)) {
            return false;
        } else {
            return true;
        }
    }

    void set_working_mode(int mode) {
        LidarWorkingMode working_mode = static_cast<LidarWorkingMode>(mode);
        lreader->setLidarWorkingMode(working_mode);
    }

    static void signal_handler(int signal) {
        if (signal == SIGINT) {
            std::cout << "\nProgram interrupted\n" << std::endl;
            exit(0); 
        }
    }

    std::string get_firmware_version() {
        while (true) {

            if (lreader->runParse() == VERSION) {
                return lreader->getVersionOfFirmware();
            }
        }
    }

    std::string get_sdk_version() {
        return lreader->getVersionOfSDK();
    }

    float get_dirty_percentage() {
        while (true) {
            if (lreader->runParse() == AUXILIARY) {
                return lreader->getDirtyPercentage();
            }
        }
    }

    void set_led_display_mode(std::vector<uint8_t> led_table) {
        lreader->setLEDDisplayMode(led_table.data());
    }

    py::dict get_imu_data() {
        py::dict imu_data;

        imu_data["timestamp"] = lreader->getIMU().stamp;
        imu_data["id"] = lreader->getIMU().id;
        imu_data["quaternion"] = py::make_tuple(lreader->getIMU().quaternion[0],
                                                lreader->getIMU().quaternion[1],
                                                lreader->getIMU().quaternion[2],
                                                lreader->getIMU().quaternion[3]);
        return imu_data;
    }

    py::dict get_cloud_data() {
        py::dict cloud_data;
        cloud_data["timestamp"] = lreader->getCloud().stamp;

        
        py::list points_list;
        Point3D temp_point;
        std::vector<Point3D> raw_cloud_xyz;

        for (const auto& point : lreader->getCloud().points) {
            points_list.append(py::make_tuple(point.x, point.y, point.z, point.intensity, point.time, point.ring));
            
            temp_point.x = point.x;
            temp_point.y = point.y;
            temp_point.z = point.z;
            raw_cloud_xyz.push_back(temp_point);
        }


        cloud_data["points"] = points_list;  // Assigning the list to the dict 

        if(init_ref == false) {
            ref = convertToDataPoints(raw_cloud_xyz);
            init_ref = true;
            }
        else {
            
        auto complet_cloud = calculateNormalsSet(raw_cloud_xyz);

        auto data = convertToDataPoints<float>(complet_cloud);
        
        auto icp_result = icp_simple(data, ref);

        py::array_t<float> icp_matrix({4, 4}, icp_result.first.data());

        cloud_data["icp"] = icp_matrix;
        
        ref = icp_result.second;
        }

        return cloud_data;
    }

    std::string message_type_to_string(MessageType type) {
        switch (type) {
            case IMU:
                return "IMU";
            case POINTCLOUD:
                return "POINTCLOUD";
            
            default:
                return "UNKNOWN";
        }
    }

    std::string check_message() {
        MessageType result = lreader->runParse();  // Get the message type
        return message_type_to_string(result);  // Convert to string
    }
        
    
};

// Python binding
PYBIND11_MODULE(unitree_lidar_sdk_pybind, m) {
    py::module::import("numpy");
    py::class_<UnitreeLidarWrapper>(m, "UnitreeLidarWrapper")
        .def(py::init<>())
        .def("initialize", &UnitreeLidarWrapper::initialize)
        .def("set_working_mode", &UnitreeLidarWrapper::set_working_mode)
        .def("get_firmware_version", &UnitreeLidarWrapper::get_firmware_version)
        .def("get_sdk_version", &UnitreeLidarWrapper::get_sdk_version)
        .def("get_dirty_percentage", &UnitreeLidarWrapper::get_dirty_percentage)
        .def("set_led_display_mode", &UnitreeLidarWrapper::set_led_display_mode)
        .def("check_message", &UnitreeLidarWrapper::check_message)
        .def("get_imu_data", &UnitreeLidarWrapper::get_imu_data)
        .def("get_cloud_data", &UnitreeLidarWrapper::get_cloud_data);
}