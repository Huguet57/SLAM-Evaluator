#ifndef LIBRARIES_H
#define LIBRARIES_H
#include "Common.hpp"
#include "Utils.hpp"
#include "Logger.hpp"
#endif

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_recorder");
    ros::NodeHandle nh;

    Logger log(nh);

    ros::Rate rate(1);

    ROS_INFO("Initialized.");

    while (ros::ok()) {
        if (log.ended()) break;

        // std::cout << "[Saving...] Odom size: " << log.n_odoms() << std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    log.save();
}