#pragma once

#include <rclcpp/rclcpp.hpp>
#include <arm_controller/arm.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/msg/joint_state.hpp>

class ArmVisualizer {
public:
    ArmVisualizer(rclcpp::Node::SharedPtr node);
    ~ArmVisualizer();

    void publishTransforms(Arm& arm, double angle1, double angle2, double angle3, const std::string& frame_prefix);
    void createMarkers(const std::tuple<Vector3d, Vector3d, Vector3d>& pos, const std::string& base_frame, 
                      const std::string& ns_prefix, int arm_id, visualization_msgs::msg::MarkerArray& marker_array);
    void publishBaseLinkTransform(const Eigen::Vector3d& body_pos, const Eigen::Quaterniond& body_q);
    void publishStaticTransforms();
    void publishMarkers(const visualization_msgs::msg::MarkerArray& marker_array);
    void publishSplineWaypoints(const visualization_msgs::msg::Marker& spline_marker);
    void publishJointStates();

    sensor_msgs::msg::JointState goal_joint_state;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_target_angle_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_spline_waypoints_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    
    std::string frame_prefix_;
    std::string frame_prefix_goal_;
};