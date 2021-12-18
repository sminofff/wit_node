#include "wit_node/wit_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace wit
{

    WitNode::WitNode(const rclcpp::NodeOptions& options)
    : WitNode::WitNode("", options)
    {}

    WitNode::WitNode(const std::string &node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node("wit_ros", node_name, options)
    {
        if(this->init() == false){
            rclcpp::shutdown();
        };
        rclcpp::CallbackGroup::SharedPtr group = nullptr;
        std::chrono::duration<double> process_rate_(1.0 / this->publish_hz_);
        this->update_timer_ = this->create_wall_timer(
            process_rate_,
            std::bind(&WitNode::update, this),
            group
        );
    }
    WitNode::~WitNode()
    {
        RCLCPP_INFO(this->get_logger(), 
                    "Waiting for WitDriver finish.");
        // update_thread_.join();
        wd_->shutdown();
    }
    void WitNode::initializeParameter()
    {
        this->wit_param_.port_ = this->declare_parameter<std::string>("port", this->wit_param_.port_);
        this->wit_param_.baud_rate_ = this->declare_parameter<int>("baud_rate", this->wit_param_.baud_rate_);
        this->frame_id_ = this->declare_parameter<std::string>("frame_id", this->frame_id_);
        this->publish_hz_ = this->declare_parameter<double>("publish_hz", this->publish_hz_);

        RCLCPP_INFO(this->get_logger(), 
                    "port: " + this->wit_param_.port_);
        RCLCPP_INFO(this->get_logger(), 
                    "baud_rate: " + std::to_string(this->wit_param_.baud_rate_));
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
            wd_ = std::make_unique<wit::WitDriver>();
            wd_->init(this->wit_param_);
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
        // update_thread_.start(&wit::WitDriver::spin, *this->wd_);
        RCLCPP_INFO(this->get_logger(),
                    "Initialized!");
        return true;
    }

    void WitNode::update()
    {
        if (wd_->isShutdown())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Driver has been shutdown. Stopping update loop.");
            rclcpp::shutdown();
            return;
        }
        if (!wd_->isConnected())
        {
            RCLCPP_ERROR(
                this->get_logger(), 
                "arm serial port is not connetced, please connect the arm.");
            rclcpp::shutdown();
            return;
        }
        this->processStreamData();
    }

    void WitNode::createPublishers()
    {
        this->imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        this->gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps", 10);
        this->raw_data_pub_ = this->create_publisher<wit_msgs::msg::ImuGpsRaw>("~/raw_data", 10);
        this->related_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/related_yaw", 10);
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
        wd_->resetYawOffset();
    }

    void WitNode::processStreamData()
    {
        std_msgs::msg::Header header;
        header.frame_id = this->frame_id_;
        header.stamp = rclcpp::Clock().now();
        Data::IMUGPS data = wd_->getData();
        
        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = wd_->getRelatedYaw();

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header = header;
        imu_msg.angular_velocity.x = data.w[0];
        imu_msg.angular_velocity.y = data.w[1];
        imu_msg.angular_velocity.z = data.w[2];

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

        sensor_msgs::msg::NavSatFix gps_msg;
        gps_msg.header = header;
        gps_msg.altitude = data.altitude;
        gps_msg.latitude = data.latitude;
        gps_msg.longitude = data.longtitude;
        gps_msg.position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

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

        this->imu_pub_->publish(imu_msg);
        this->gps_pub_->publish(gps_msg);
        this->raw_data_pub_->publish(raw_msg);
        this->related_yaw_pub_->publish(yaw_msg);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<wit::WitNode>(node_options));
  rclcpp::shutdown();
  return 0;
}

// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(wit::WitNode)