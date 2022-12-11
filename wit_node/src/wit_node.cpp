#include "wit_node/wit_node.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

namespace wit
{

    WitNode::WitNode(const rclcpp::NodeOptions& options)
    : WitNode::WitNode("", options)
    {}

    WitNode::WitNode(const std::string &node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node("wit_ros", node_name, options)
    {
        if(this->init() == false){
            rclcpp::shutdown(nullptr);
        };
    }
    WitNode::~WitNode()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for WitDriver finish.");
        wd->shutdown();
    }
    void WitNode::initializeParameter()
    {
        this->wit_param_.port_ = this->declare_parameter<std::string>("port", this->wit_param_.port_);
        this->wit_param_.baud_rate_ = this->declare_parameter<int>("baud_rate", this->wit_param_.baud_rate_);
        this->frame_id = this->declare_parameter<std::string>("frame_id", this->frame_id);
        this->publish_hz = this->declare_parameter<double>("publish_hz", this->publish_hz);

        RCLCPP_INFO(this->get_logger(), 
                    "port: " + this->wit_param_.port_);
        RCLCPP_INFO(this->get_logger(), 
                    "baud_rate: " + std::to_string(this->wit_param_.baud_rate_));
        RCLCPP_INFO(this->get_logger(), 
                    "frame_id: " + this->frame_id);
        RCLCPP_INFO(this->get_logger(), 
                    "publish_hz: " + std::to_string(this->publish_hz));
    }
    bool WitNode::init()
    {
        RCLCPP_INFO(this->get_logger(),
                    "initializing...");
        this->initializeParameter();
        this->createPublishers();
        this->createSubscrptions();

        /*********************
         ** Driver Init
        **********************/
        try
        {
            wd = std::make_unique<wit::WitDriver>();
            wd->init(this->wit_param_);
            rclcpp::WallRate rate(100ms);
            rate.sleep();
        }
        catch (const ecl::StandardException &e)
        {
            switch (e.flag())
            {
            case (ecl::OpenError):
            {

                RCLCPP_ERROR(
                    this->get_logger(), 
                    "Could not open connection ["+ this->wit_param_.port_ + "].");
                break;
            }
            default:
            {
                RCLCPP_ERROR(
                    this->get_logger(), 
                    "Initialization failed. Please restart.");
                RCLCPP_ERROR(
                    this->get_logger(), 
                    e.what());
                break;
            }
            }
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Initialized!");
        return true;
    }

    void WitNode::createPublishers()
    {
        this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS());
        this->gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps", rclcpp::SensorDataQoS());
        this->raw_data_pub = this->create_publisher<wit_msgs::msg::ImuGpsRaw>("~/raw_data", rclcpp::SensorDataQoS());
        this->related_yaw_pub = this->create_publisher<std_msgs::msg::Float64>("~/related_yaw", rclcpp::SensorDataQoS());
        this->imu_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/imu_pose", rclcpp::SensorDataQoS());
    }

    void WitNode::createSubscrptions()
    {
        this->reset_offset_sub_ = create_subscription<std_msgs::msg::Empty>(
            "~/reset_offset", 10,
            std::bind(&WitNode::subscribeResetOffset,
                      this, std::placeholders::_1));
    }
    void WitNode::subscribeResetOffset(const std_msgs::msg::Empty::ConstSharedPtr msg)
    {
        wd->resetYawOffset();
        RCLCPP_INFO(this->get_logger(), "~/resetYawOffset.");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<wit::WitNode>(node_options);
    rclcpp::WallRate rate(node->publish_hz);
    while (rclcpp::ok())
    {
        if (node->wd->isShutdown())
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Driver has been shutdown. Stopping update loop.");
            break;
        }
        if (!node->wd->isConnected())
        {
            RCLCPP_ERROR(
                node->get_logger(), 
                "arm serial port is not connetced, please connect the arm.");
            break;
        }

        std_msgs::msg::Header header;
        header.frame_id = node->frame_id;
        header.stamp = rclcpp::Clock().now();
        wit::Data::IMUGPS data = node->wd->getData();
        
        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = node->wd->getRelatedYaw();

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header = header;
        // gyro
        imu_msg.angular_velocity.x = data.w[0];
        imu_msg.angular_velocity.y = data.w[1];
        imu_msg.angular_velocity.z = data.w[2];
        // acc
        imu_msg.linear_acceleration.x = data.a[0];
        imu_msg.linear_acceleration.y = data.a[1];
        imu_msg.linear_acceleration.z = data.a[2];

        tf2::Quaternion q_tf2;
        q_tf2.setRPY(data.rpy[0], data.rpy[1], data.rpy[2]);
        imu_msg.orientation = tf2::toMsg(q_tf2);
        imu_msg.orientation_covariance =
            {0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.001};
        imu_msg.angular_velocity_covariance =
            {0.00001, 0.0, 0.0,
                0.0, 0.00001, 0.0,
                0.0, 0.0, 0.00001};
        imu_msg.linear_acceleration_covariance =
            {0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01};

        geometry_msgs::msg::PoseStamped imu_pose;
        imu_pose.header = header;
        imu_pose.pose.orientation = imu_msg.orientation;
        
        sensor_msgs::msg::NavSatFix gps_msg;
        gps_msg.header = header;
        gps_msg.altitude = data.altitude;
        gps_msg.latitude = data.latitude;
        gps_msg.longitude = data.longtitude;
        gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        wit_msgs::msg::ImuGpsRaw raw_msg;
        raw_msg.header = header;
        for (int i = 0; i < 3; i++)
        {
            raw_msg.acc.push_back(data.a[i]);
            raw_msg.gyro.push_back(data.w[i]);
            raw_msg.rpy.push_back(data.rpy[i]);
            raw_msg.mag.push_back(data.mag[i]);
            raw_msg.dop.push_back(data.gpsa[i]);
        }
        for (int i = 0; i < 4; i++)
        {
            raw_msg.ps.push_back(data.d[i]);
            raw_msg.quarternion.push_back(data.q[i]);
        }
        raw_msg.sn = data.satelites;
        raw_msg.gpsh = data.gpsh;
        raw_msg.gpsy = data.gpsy;
        raw_msg.gpsv = data.gpsv;
        raw_msg.ap = data.pressure;
        raw_msg.longtitude = data.longtitude;
        raw_msg.altitude = data.altitude;
        raw_msg.latitude = data.latitude;
        raw_msg.time = data.timestamp;
        raw_msg.temperature = data.temperature;

        node->imu_pub->publish(imu_msg);
        node->gps_pub->publish(gps_msg);
        node->raw_data_pub->publish(raw_msg);
        node->related_yaw_pub->publish(yaw_msg);
        node->imu_pose_pub->publish(imu_pose);

        if(!rclcpp::ok()) {
            break;
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown(nullptr);
    return 0;
}
