#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp/behavior_tree.h"

//using namespace std::chrono_literals;

class SpinInPlace : public BT::StatefulActionNode
{
  public:

    //rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Node::SharedPtr node_s_;
    rclcpp::Time start_time_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    geometry_msgs::msg::Twist vel_msgs;
    int spin_time__;

    //client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "navigate_to_pose");

    // Method overrides
    SpinInPlace(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr):
        BT::StatefulActionNode(name, config),  node_s_{node_ptr}{}

    BT::NodeStatus onStart() override
    {
        float speed__;
        
        if (!getInput<float>("speed", speed__)) {
            throw BT::RuntimeError("missing required input [goal]");
        }
        if (!getInput<int>("spin_time", spin_time__)) {
            throw BT::RuntimeError("missing required input [goal]");
        }
        vel_pub_ = node_s_->create_publisher<geometry_msgs::msg::Twist>("/bt_vel", 10);
        //std::cout << "[" << this->name() << "] " << goal_.x<<" ,"<<goal_.y<<" ,"<<goal_.theta_a<<" !!" << std::endl;
        vel_msgs.angular.z = speed__;
        vel_pub_->publish(vel_msgs);
        start_time_ = node_s_->now();
        
  
      
      //std::cout << "[" << this->name() << "] Sent goal message" << std::endl;
      return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override
    {
        auto elapsed = node_s_->now() - start_time_;
        if (elapsed < std::chrono::seconds(spin_time__)) {
            //std::cout << "[" << this->name() << "] Goal reached" << std::endl;
            vel_pub_->publish(vel_msgs);
            return BT::NodeStatus::RUNNING;
                          
        } else {
            return BT::NodeStatus::SUCCESS;
        }
    
    }
    void onHalted() override {};
    static BT::PortsList providedPorts() { 
        return { BT::InputPort<float>("speed"),
             BT::InputPort<int>("spin_time") 
        }; 
    };
};