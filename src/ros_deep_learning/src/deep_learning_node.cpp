#include <ros/ros.h>
#include "node_detectnet.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "detectnet");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    detect_object::DetectObject detect_object(nh,private_nh);
    if (!detect_object.initializeNode()) {
        ROS_ERROR("CajaLidarSafety - main, failed to init, exit");
        return 1;
    }
    ROS_INFO("CajaLidarSafety - main, init successfully");
    ros::spin();

    return 0;
}
