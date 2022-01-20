struct LoggerParams {
    std::string odom_topic, end_topic, gt_topic, world_frame_id, directory, filename, extension;
};

class Logger {
    public:
        LoggerParams params;

        Logger(ros::NodeHandle& nh) {
            this->fill_parameters(nh);
            this->interpret_parameters();
            this->create_folder(params.directory);

            this->subscribe_to_odom(nh);
            this->subscribe_to_ended(nh);
            this->subscribe_to_gt(nh);
        }

        int n_odoms() {
            return this->odoms.size();
        }

        void save() {
            // Save to file
            std::ofstream odom_file;
            odom_file.open(this->path_to_save, std::ios::out);
            this->odoms_to_file(odom_file, this->odoms);
            odom_file.close();

            // Confirmation
            ROS_WARN("Saved in: %s", this->path_to_save.c_str());
        }

        bool ended() {
            return this->has_ended;
        }

        void add_odom(const nav_msgs::Odometry& tf) {
            odoms.push_back(tf);
        }

    private:
        ros::Subscriber odom_sub;
        ros::Subscriber end_sub;
        ros::Subscriber gt_sub;

        bool has_ended = false;
        std::vector<nav_msgs::Odometry> odoms;
        std::string path_to_save;

        void subscribe_to_ended(ros::NodeHandle& nh) { this->end_sub = nh.subscribe(params.end_topic, 1000, &Logger::callback_end, this); }
        void callback_end(const std_msgs::Bool::ConstPtr& end) { this->has_ended = end->data; }
        
        void subscribe_to_odom(ros::NodeHandle& nh) { this->odom_sub = nh.subscribe(params.odom_topic, 1000, &Logger::callback_odom, this); }
        void callback_odom(const nav_msgs::Odometry::ConstPtr& tf) { this->add_odom(*tf); }

        void subscribe_to_gt(ros::NodeHandle& nh) { this->gt_sub = nh.subscribe(params.gt_topic, 1000, &Logger::callback_tf, this); }
        void callback_tf(const tf2_msgs::TFMessage::ConstPtr& msg) { for (nav_msgs::Odometry odom : this->get_odoms_for_frame(msg, params.world_frame_id)) this->add_odom(odom); }

        void fill_parameters(ros::NodeHandle& nh) {
            nh.param<std::string>("odom_topic", params.odom_topic, "/algorithm/state");
            nh.param<std::string>("end_topic", params.end_topic, "/algorithm/end");
            nh.param<std::string>("gt_topic", params.gt_topic, "/tf");
            nh.param<std::string>("world_frame_id", params.world_frame_id, "world");
            nh.param<std::string>("directory", params.directory, "/path/to/data/");
            nh.param<std::string>("filename", params.filename, "new_commit");
            nh.param<std::string>("extension", params.extension, ".csv");
        }

        void interpret_parameters() {
            this->path_to_save = params.directory + params.filename + params.extension;
        }

        void create_folder(std::string path_to_folder){
            if (not std::filesystem::is_directory(path_to_folder) or not std::filesystem::exists(path_to_folder)) {
                std::filesystem::create_directory(path_to_folder);
                ROS_WARN("Created folder: %s", path_to_folder.c_str());
            }   
        }

        void odoms_to_file(std::ofstream& odom_file, const std::vector<nav_msgs::Odometry>& odoms) {
            for (const nav_msgs::Odometry& odom : odoms) {
                odom_file << std::setprecision(16)
                        << odom.header.stamp.toSec() << ","
                        << odom.pose.pose.position.x << ","
                        << odom.pose.pose.position.y << ","
                        << odom.pose.pose.position.z << ","
                        << odom.pose.pose.orientation.x << ","
                        << odom.pose.pose.orientation.y << ","
                        << odom.pose.pose.orientation.z << ","
                        << odom.pose.pose.orientation.w << std::endl;
            }
        }

        std::vector<nav_msgs::Odometry> get_odoms_for_frame(const tf2_msgs::TFMessage::ConstPtr& msg, const std::string& world_frame) {
            std::vector<nav_msgs::Odometry> res;

            for (auto tf : msg->transforms)
                if (tf.header.frame_id == world_frame)
                    res.push_back(this->tf_to_odom(tf));

            return res;
        }

        nav_msgs::Odometry tf_to_odom(const geometry_msgs::TransformStamped& tf) {
            nav_msgs::Odometry msg;
            msg.header = tf.header;

            auto t = tf.transform.translation;
            Eigen::Vector3d tnow(t.x, t.y, t.z);
            msg.pose.pose.position.x = tnow.x();
            msg.pose.pose.position.y = tnow.y();
            msg.pose.pose.position.z = tnow.z();

            auto q = tf.transform.rotation;
            Eigen::Quaterniond qnow(q.w, q.x, q.y, q.z);
            msg.pose.pose.orientation.x = qnow.x();
            msg.pose.pose.orientation.y = qnow.y();
            msg.pose.pose.orientation.z = qnow.z();
            msg.pose.pose.orientation.w = qnow.w();
            
            return msg;
        }
};