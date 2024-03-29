//#pragma once

//#include <chrono>
//#include <iomanip>
#include <iostream>
//#include <memory>
#include <string>
//#include <random>

//#include "rclcpp/rclcpp.hpp"
//#include "rclcpp_action/rclcpp_action.hpp"

//#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "std_msgs/msg/header.hpp"
//#include "geometry_msgs/msg/pose.hpp"
//#include "geometry_msgs/msg/quaternion.hpp"
//#include "nav2_msgs/action/navigate_to_pose.hpp"
//#include "tf2/transform_datatypes.h"
//#include "tf2/LinearMath/Quaternion.h"
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "behaviortree_cpp/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp/behavior_tree.h"


//Custom type
struct Pose2D
{
    double x, y, theta_a;
};


namespace BT
{
template <> inline
Pose2D convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x       = convertFromString<double>(parts[0]);
        output.y       = convertFromString<double>(parts[1]);
        output.theta_a = convertFromString<double>(parts[2]);
	return output;
    }
}
} // end namespace BT


class GoToPose : public BT::StatefulActionNode
{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    bool done_flag_;
    Pose2D goal_;
    rclcpp_action::ResultCode nav_result_;
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

    //client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "navigate_to_pose");

    // Method overrides
    GoToPose(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr):
        BT::StatefulActionNode(name, config),  node_ptr_{node_ptr}{}

    virtual BT::NodeStatus onStart() override
    {
      auto target_loc = getInput<Pose2D>("goal", goal_);
      if (!target_loc) {
          std::cerr << "Couldn't get target loc!" << std::endl;
          return BT::NodeStatus::FAILURE;
      }
      // Set up the action client
      using namespace std::placeholders;
      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(&GoToPose::result_callback, this, _1);
      client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "navigate_to_pose");
      std::cout << "[" << this->name() << "] " << goal_.x<<" ,"<<goal_.y<<" ,"<<goal_.theta_a<<" !!" << std::endl;
      // // Package up the the goal
      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.pose.position.x = goal_.x;
      goal_msg.pose.pose.position.y = goal_.y;
      tf2::Quaternion q;
      q.setRPY(0, 0, goal_.theta_a);
      q.normalize();
      goal_msg.pose.pose.orientation = tf2::toMsg(q);

      std::cout << "[" << this->name() << "] "<<goal_msg.pose.pose.orientation.x<<" "<<goal_msg.pose.pose.orientation.y<<" "
      <<goal_msg.pose.pose.orientation.z<<" "
      <<goal_msg.pose.pose.orientation.w<< std::endl;
      // Send the navigation action goal.
      done_flag_ = false;
      client_ptr_->async_send_goal(goal_msg, send_goal_options);
      
      std::cout << "[" << this->name() << "] Sent goal message" << std::endl;
      return BT::NodeStatus::RUNNING;
    }
    virtual BT::NodeStatus onRunning() override
    {
      if (done_flag_) {
      if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
      std::cout << "[" << this->name() << "] Goal reached" << std::endl;
          return BT::NodeStatus::SUCCESS;   
          } else {
              std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
              return BT::NodeStatus::FAILURE;   
          }
      } else {
          return BT::NodeStatus::RUNNING;
      }
        // std::cout << "[" << this->name() << "] Sent goal success" << std::endl;
        // return BT::NodeStatus::SUCCESS;
    }
    virtual void onHalted() override {};
    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<Pose2D>("goal")};
    }

    // Action client callbacks
    void result_callback(const GoalHandleNav::WrappedResult& result)
    {
      if (result.result) {
        done_flag_ = true;
        nav_result_ = result.code;
      }
    }
  //private:
    //rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    // using namespace std::placeholders;
    // auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // send_goal_options.result_callback = std::bind(&GoToPose::result_callback, this, _1);
    // client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "navigate_to_pose");
};
//-------------------------

