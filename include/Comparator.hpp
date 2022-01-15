class Comparator {
    Comparator() {}

    void evaluate() {
        if (gts.empty() or ours.empty()) return; 
        std::cout << "gts: " << gts.size() << std::endl;
        std::cout << "ours: " << ours.size() << std::endl;

        int Op = 0;
        std::vector<nav_msgs::Odometry> our_ints;

        // Calculate interpolated values (so gt.time == our.time)
        for (const nav_msgs::Odometry& gt : gts) {
            if (gt.header.stamp.toSec() < ours.front().header.stamp.toSec()) continue;
            while (Op < ours.size() and ours[Op].header.stamp.toSec() < gt.header.stamp.toSec()) ++Op;
            
            if (Op >= ours.size()) break;
            if (ours[Op].header.stamp.toSec() < gt.header.stamp.toSec()) break;
            if (gt.header.stamp.toSec() < ours[Op-1].header.stamp.toSec()) break;

            nav_msgs::Odometry our_int = this->interpolate(ours[Op-1], ours[Op], gt.header.stamp.toSec());
            our_ints.push_back(our_int);
        }

        Op = 1;
        double pos_error = 0.d;
        double euler_error = 0.d;
        double total_pos_gt = 0.d;
        double total_quat_gt = 0.d;

        std::ofstream ours_file, gt_file, res_file, pos_file, quat_file;
        ours_file.open("/media/andreu/Movies/ws_limovelo/data/newest/odometries_ours.csv", ios::out);
        gt_file.open("/media/andreu/Movies/ws_limovelo/data/newest/odometries_gt.csv", ios::out);
        res_file.open("/media/andreu/Movies/ws_limovelo/data/newest/results.csv", ios::out);
        pos_file.open("/media/andreu/Movies/ws_limovelo/data/newest/pos_errors.csv", ios::out);
        quat_file.open("/media/andreu/Movies/ws_limovelo/data/newest/quat_errors.csv"), ios::out;

        // Evaluate difference of differences (no drift)
        for (int i = 1; i < gts.size(); ++i) {
            while (Op < our_ints.size() and our_ints[Op].header.stamp.toSec() < gts[i].header.stamp.toSec()) ++Op;
            if (Op < our_ints.size() and gts[i].header.stamp.toSec() < our_ints[Op].header.stamp.toSec()) continue;
            if (Op >= our_ints.size() or our_ints[Op].header.stamp.toSec() != gts[i].header.stamp.toSec()) break;

            Eigen::Vector3d gt_dpos = this->diff_pos(gts[i-1], gts[i]);
            Eigen::Quaterniond gt_dquat = this->diff_quat(gts[i-1], gts[i]);

            Eigen::Vector3d our_dpos = this->diff_pos(our_ints[Op-1], our_ints[Op]);
            Eigen::Quaterniond our_dquat = this->diff_quat(our_ints[Op-1], our_ints[Op]);

            auto ori_gt = gts[i-1].pose.pose.orientation;
            auto ori_ours = our_ints[Op-1].pose.pose.orientation;
            Eigen::Quaterniond q_gt(ori_gt.w, ori_gt.x, ori_gt.y, ori_gt.z);
            Eigen::Quaterniond q_ours(ori_ours.w, ori_ours.x, ori_ours.y, ori_ours.z);

            auto pos_gt = gts[i-1].pose.pose.position;
            auto pos_ours = our_ints[Op-1].pose.pose.position;
            double timestamp_tminus1 = gts[i-1].header.stamp.toSec();

            ours_file << std::setprecision(16) << our_ints[Op-1].header.stamp.toSec() << "," << pos_ours.x << "," << pos_ours.y << "," << pos_ours.z << "," << q_ours.x() << "," << q_ours.y() << "," << q_ours.z() << "," << q_ours.w() << std::endl;
            gt_file << std::setprecision(16) << gts[i-1].header.stamp.toSec() << ","  << pos_gt.x << "," << pos_gt.y << "," << pos_gt.z << "," << q_gt.x() << "," << q_gt.y() << "," << q_gt.z() << "," << q_gt.w() << std::endl;
            pos_file << std::setprecision(16) << timestamp_tminus1 << "," << (q_gt.conjugate()*gt_dpos - q_ours.conjugate()*our_dpos)(0) << "," << (q_gt.conjugate()*gt_dpos - q_ours.conjugate()*our_dpos)(1) << "," << (q_gt.conjugate()*gt_dpos - q_ours.conjugate()*our_dpos)(2) << std::endl;
            quat_file << std::setprecision(16) << timestamp_tminus1 << "," << this->euler_diff(our_dquat, gt_dquat)(0) << "," << this->euler_diff(our_dquat, gt_dquat)(1) << "," << this->euler_diff(our_dquat, gt_dquat)(2) << std::endl;

            pos_error += (q_gt.conjugate()*gt_dpos - q_ours.conjugate()*our_dpos).norm();
            euler_error += this->euler_diff(our_dquat, gt_dquat).norm();
            
            total_pos_gt += gt_dpos.norm();
            total_quat_gt += this->euler_diff(gts[i-1], gts[i]).norm();

            res_file << std::setprecision(16) << timestamp_tminus1 << "," << (q_gt.conjugate()*gt_dpos - q_ours.conjugate()*our_dpos).norm() << "," << gt_dpos.norm() << "," << pos_error/total_pos_gt*100 << std::endl;
        }

        ours_file.close();
        gt_file.close();
        pos_file.close();
        quat_file.close();

        res_file << std::setprecision(16) << "0.0," << pos_error << "," << total_pos_gt << "," << pos_error/total_pos_gt*100 << std::endl;
        res_file.close();

        std::cout << "/////////" << std::endl;
        std::cout << "POS: " << pos_error << " / " << total_pos_gt << " (= " << pos_error/total_pos_gt*100 << "%)" << std::endl;
        std::cout << "ROT: " << euler_error << " / " << total_quat_gt << " (= " << euler_error/total_quat_gt*100 << "%)" << std::endl;
        std::cout << "/////////" << std::endl;
    }
};