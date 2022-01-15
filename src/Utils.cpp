#ifndef LIBRARIES_H
#define LIBRARIES_H
#include "Common.hpp"
#include "Utils.hpp"
#include "Logger.hpp"
#endif

Eigen::Vector3d OdomMath::diff_pos(const nav_msgs::Odometry& od1, const nav_msgs::Odometry& od2) {
    auto pose1 = od1.pose.pose.position;
    auto pose2 = od2.pose.pose.position;
    
    Eigen::Vector3d v1(pose1.x, pose1.y, pose1.z);
    Eigen::Vector3d v2(pose2.x, pose2.y, pose2.z);

    return v2 - v1;
}

Eigen::Quaterniond OdomMath::diff_quat(const nav_msgs::Odometry& od1, const nav_msgs::Odometry& od2) {
    auto ori1 = od1.pose.pose.orientation;
    auto ori2 = od2.pose.pose.orientation;

    Eigen::Quaterniond q1(ori1.w, ori1.x, ori1.y, ori1.z); 
    Eigen::Quaterniond q2(ori2.w, ori2.x, ori2.y, ori2.z);

    return q2 * q1.conjugate();
}

Eigen::Vector3d OdomMath::interpolate_pos(const nav_msgs::Odometry& od1, const nav_msgs::Odometry& od2, double alpha) {
    auto pose1 = od1.pose.pose.position;
    auto pose2 = od2.pose.pose.position;
    
    Eigen::Vector3d v1(pose1.x, pose1.y, pose1.z);
    Eigen::Vector3d v2(pose2.x, pose2.y, pose2.z);

    return alpha*v1 + (1-alpha)*v2;
}

Eigen::Quaterniond OdomMath::slerp_quat(const nav_msgs::Odometry& od1, const nav_msgs::Odometry& od2, double alpha) {
    auto ori1 = od1.pose.pose.orientation;
    auto ori2 = od2.pose.pose.orientation;

    Eigen::Quaterniond q1(ori1.w, ori1.x, ori1.y, ori1.z); 
    Eigen::Quaterniond q2(ori2.w, ori2.x, ori2.y, ori2.z);

    return q1.slerp(1 - alpha, q2);
}

// Interpolation: a*OD1 + (1-a)*OD2
nav_msgs::Odometry OdomMath::interpolate(const nav_msgs::Odometry& od1, const nav_msgs::Odometry& od2, double t) {            
    nav_msgs::Odometry od_int;
    od_int.header.stamp = ros::Time(t);
    double alpha = (od2.header.stamp.toSec() - t)/(od2.header.stamp.toSec() - od1.header.stamp.toSec());
    assert ( ("t is not between OD1.time and OD2.time", 0 <= alpha and alpha <= 1) );

    Eigen::Vector3d int_pos = this->interpolate_pos(od1, od2, alpha);
    od_int.pose.pose.position.x = int_pos(0);
    od_int.pose.pose.position.y = int_pos(1);
    od_int.pose.pose.position.z = int_pos(2);

    Eigen::Quaterniond dquat = this->slerp_quat(od1, od2, alpha);
    od_int.pose.pose.orientation.w = dquat.w();
    od_int.pose.pose.orientation.y = dquat.x();
    od_int.pose.pose.orientation.z = dquat.y();
    od_int.pose.pose.orientation.z = dquat.z();

    return od_int;
}