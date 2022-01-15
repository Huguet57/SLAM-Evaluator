namespace OdomMath {
    Eigen::Vector3d diff_pos(const nav_msgs::Odometry& od1, const nav_msgs::Odometry& od2);
    Eigen::Quaterniond diff_quat(const nav_msgs::Odometry& od1, const nav_msgs::Odometry& od2);
    Eigen::Vector3d interpolate_pos(const nav_msgs::Odometry& od1, const nav_msgs::Odometry& od2, double alpha);
    Eigen::Quaterniond slerp_quat(const nav_msgs::Odometry& od1, const nav_msgs::Odometry& od2, double alpha);
    nav_msgs::Odometry interpolate(const nav_msgs::Odometry& od1, const nav_msgs::Odometry& od2, double t);
}