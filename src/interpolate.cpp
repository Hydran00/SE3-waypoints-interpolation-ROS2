#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "pinocchio/multibody/liegroup/liegroup.hpp"

#include <vector>
#include <cmath>
#define MAX_VELOCITY 0.02
using std::placeholders::_1;
using namespace pinocchio;
// defining interpolator operator
typedef double Scalar;
typedef SpecialEuclideanOperationTpl<3, Scalar> SE3Operation;
SE3Operation aSE3;
class Interpolator : public rclcpp::Node
{
public:
    Interpolator() : Node("interpolator")
    {
        publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/spline", 10);
        waypoints_subscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/desired_waypoints",
            10,
            std::bind(&Interpolator::interpolate, this, _1));
        tool0_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tool0_pose",
            10,
            std::bind(&Interpolator::set_ee_pose, this, _1));
        RCLCPP_INFO(this->get_logger(), "Node is running, ready to interpolate trajectories");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tool0_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_subscription;
    geometry_msgs::msg::Pose end_effector_pose;
    bool received_end_effector_pose = false;
    float z_offset = 1.0;

    void set_ee_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!received_end_effector_pose)
        {
            RCLCPP_INFO(this->get_logger(), "Received end effector pose");
            received_end_effector_pose = true;
            end_effector_pose = msg->pose;
            // std::cout << "End effector pose: "<< msg->pose;
        }
        end_effector_pose = msg->pose;
    }

    void interpolate(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (!received_end_effector_pose)
        {
            RCLCPP_INFO(this->get_logger(), "Pose of the end effector not received yet, cannot compute full trajectory.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Received waypoints, computing trajectory...");

        // adding the offset to the z coordinate relative to the base_link frame
        end_effector_pose.position.z += z_offset;

        // computing the trajectory
        geometry_msgs::msg::PoseArray trajectory;
        geometry_msgs::msg::PoseArray waypoints;
        waypoints.poses.push_back(end_effector_pose);
        for (int i = 0; i < msg->poses.size(); i++)
        {
            waypoints.poses.push_back(msg->poses[i]);
        }
        for (int i = 0; i < waypoints.poses.size() - 1; i++)
        {

            int steps_num = compute_number_of_steps(waypoints.poses[i], waypoints.poses[i + 1]);

            SE3Operation::ConfigVector_t pose1, pose2;

            pose1 << waypoints.poses[i].position.x, waypoints.poses[i].position.y, waypoints.poses[i].position.z,
                waypoints.poses[i].orientation.w, waypoints.poses[i].orientation.x, waypoints.poses[i].orientation.y, waypoints.poses[i].orientation.z;

            pose2 << waypoints.poses[i + 1].position.x, waypoints.poses[i + 1].position.y, waypoints.poses[i + 1].position.z,
                waypoints.poses[i + 1].orientation.w, waypoints.poses[i + 1].orientation.x, waypoints.poses[i + 1].orientation.y, waypoints.poses[i + 1].orientation.z;

            aSE3.normalize(pose1);
            aSE3.normalize(pose2);
            // creating output vector
            SE3Operation::ConfigVector_t interpolated_waypoints;
            // interpolating
            for (int j = 0; j < steps_num; j++)
            {
                aSE3.interpolate(pose1, pose2, (float)j / steps_num, interpolated_waypoints);
                std::cout << "Interpolated configuration: " << interpolated_waypoints.transpose() << std::endl;
                // adding every point to the trajectory
                geometry_msgs::msg::Pose pose;
                pose.position.x = interpolated_waypoints[0];
                pose.position.y = interpolated_waypoints[1];
                pose.position.z = interpolated_waypoints[2];
                pose.orientation.w = interpolated_waypoints[3];
                pose.orientation.x = interpolated_waypoints[4];
                pose.orientation.y = interpolated_waypoints[5];
                pose.orientation.z = interpolated_waypoints[6];
                trajectory.poses.push_back(pose);
                std::cout << j << std::endl;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Publishing trajectory...");
        publisher->publish(trajectory);
    }
    int compute_number_of_steps(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2)
    {
        float step_num = 1;
        // compute linear distance
        float lin_distance = sqrt(
            pow(pose1.position.x - pose2.position.x, 2) +
            pow(pose1.position.y - pose2.position.y, 2) +
            pow(pose1.position.z - pose2.position.z, 2));
        std::cout << "Linear distance : " << lin_distance << std::endl;

        // compute angular distance between quaternions
        Eigen::Quaternion q1(pose1.orientation.w, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z);
        Eigen::Quaternion q2(pose2.orientation.w, pose2.orientation.x, pose2.orientation.y, pose2.orientation.z);
        float ang_distance = std::abs(q1.angularDistance(q2));
        std::cout << "Angular distance : " << ang_distance << std::endl;
        // compute max velocity between linear and angular
        float K = 1.5;
        float max_v = std::max(lin_distance, K * ang_distance) / step_num;
        std::cout << "Max velocity  : " << max_v << std::endl;
        // if (max_v < MAX_VELOCITY)
        // {
        //     std::cout << "Max velocity reached: " << max_v << std::endl;
        //     return step_num;
        // }
        //step_num++;
        step_num = std::ceil(max_v / MAX_VELOCITY);
        return step_num;
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Interpolator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
