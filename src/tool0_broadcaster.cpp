#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tool0_pose", 10);
        this->declare_parameter("ee_frame_name", "wrist_3_link");
        std::string ee_frame_name = this->get_parameter("ee_frame_name").as_string();
        RCLCPP_INFO(this->get_logger(), "ee_frame_name: %s", ee_frame_name.c_str());
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        rclcpp::Rate rate(1);
        rclcpp::Clock::SharedPtr clock = this->get_clock();
        std::string warning_msg;
        // Retrieve the transformation between source_frame and target_frame
        while (rclcpp::ok() && !tf_buffer_->canTransform(
                                   "base_link", ee_frame_name.c_str(), tf2::TimePoint(), &warning_msg))
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *clock, 1000, "Waiting for transform");
            // source_frameid.c_str(), target_frameid.c_str());
            rate.sleep();
        }
        while (rclcpp::ok())
        {
            try
            {
                geometry_msgs::msg::TransformStamped transformStamped =
                    tf_buffer_->lookupTransform("base_link", ee_frame_name, tf2::TimePointZero);
                // Extract position and orientation from the transform
                geometry_msgs::msg::Vector3 position = transformStamped.transform.translation;
                geometry_msgs::msg::Quaternion orientation = transformStamped.transform.rotation;
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "base_link";
                pose.header.stamp = this->now();
                pose.pose.position.x = position.x;
                pose.pose.position.y = position.y;
                pose.pose.position.z = position.z;
                pose.pose.orientation = orientation;
                publisher_->publish(pose);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
                return;
            }
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
