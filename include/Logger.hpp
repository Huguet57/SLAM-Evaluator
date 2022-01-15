struct LoggerParams {
    std::string odom_topic, end_topic, directory, filename, extension;
};

class Logger {
    public:
        LoggerParams params;

        Logger(ros::NodeHandle& nh) {
            this->fill_parameters(nh);
            this->interpret_parameters();
            this->subscribe_to_odom(nh);
            this->subscribe_to_ended(nh);
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
            std::cout << "Saved in: " << this->path_to_save << std::endl;
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

        bool has_ended = false;
        std::vector<nav_msgs::Odometry> odoms;
        std::string path_to_save;

        void subscribe_to_ended(ros::NodeHandle& nh) { this->end_sub = nh.subscribe(params.end_topic, 1000, &Logger::callback_end, this); }
        void callback_end(const std_msgs::Bool::ConstPtr& end) { this->has_ended = end->data; }
        
        void subscribe_to_odom(ros::NodeHandle& nh) { this->odom_sub = nh.subscribe(params.odom_topic, 1000, &Logger::callback_odom, this); }
        void callback_odom(const nav_msgs::Odometry::ConstPtr& tf) { this->add_odom(*tf); }

        void fill_parameters(ros::NodeHandle& nh) {
            nh.param<std::string>("odom_topic", params.odom_topic, "/algorithm/state");
            nh.param<std::string>("end_topic", params.end_topic, "/algorithm/end");
            nh.param<std::string>("directory", params.directory, "/path/to/data/");
            nh.param<std::string>("filename", params.filename, "new_commit");
            nh.param<std::string>("extension", params.extension, ".csv");
        }

        void interpret_parameters() {
            this->path_to_save = params.directory + params.filename + params.extension;
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
};