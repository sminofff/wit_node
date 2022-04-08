

#ifndef WIT_ROS_HPP
#define WIT_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "wit_driver/wit_driver.hpp"
#include "wit_msgs/msg/imu_gps_raw.hpp"

using namespace std;

namespace wit
{

    class WitNode : public rclcpp::Node
    {
    public:
        WitNode(const rclcpp::NodeOptions &options);
        WitNode(const std::string &node_name, const rclcpp::NodeOptions &options);
        ~WitNode();

        std::unique_ptr<WitDriver> wd;
        std::string frame_id = "imu_link";
        double publish_hz = 10.0;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
        rclcpp::Publisher<wit_msgs::msg::ImuGpsRaw>::SharedPtr raw_data_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr related_yaw_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_pub;

    private:
        bool init();

        /*********************
         ** Variables
        **********************/
        void initializeParameter();
        Parameter wit_param_;

        /*********************
         ** Ros Publishers
        **********************/
        void createPublishers();

        /*********************
         ** Ros Subscribers
        **********************/
        void createSubscrptions();
        void subscribeResetOffset(const std_msgs::msg::Empty::ConstSharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_offset_sub_;

    };
}

#endif
