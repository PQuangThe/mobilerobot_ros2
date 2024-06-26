#include <chrono>
#include <random>
//#include <string>

//#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "yaml-cpp/yaml.h"
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

//#include "nav_behavior.h"
#include "navclient2.hpp"
#include "spin_behavior.hpp"
//

const std::string default_bt_xml_file = 
    ament_index_cpp::get_package_share_directory("behavior_tree_demo_ros2") + "/behavior_tree_xml/demo_behavior_tree.xml";
// const std::string default_bt_xml_file = 
//     ament_index_cpp::get_package_share_directory("behavior_tree_demo_ros2") + "/behavior_tree_xml/test_tree.xml";


class AutonomyNode : public rclcpp::Node {
    public:
        AutonomyNode() : Node("autonomy_node") {
            this->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
            tree_xml_file_ = this->get_parameter("tree_xml_file").as_string();
            RCLCPP_INFO(this->get_logger(), "Searching for tree xml file: %s",
                    tree_xml_file_.c_str());
        }
        void execute() 
        {
            RCLCPP_INFO(this->get_logger(), "start setup behaviortree");
            factory.registerNodeType<GoToPose>("GoToPose", shared_from_this());
            factory.registerNodeType<SpinInPlace>("SpinInPlace", shared_from_this());
        
            tree_ =factory.createTreeFromFile(tree_xml_file_);//, blackboard);
            
            //tree_ = factory.createTree("Main_Tree");
            
            //publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 1668);
            BT::StdCoutLogger logger_cout(tree_);

            update_behavior_tree();
            RCLCPP_INFO(this->get_logger(), "started setup behaviortree");

            //rclcpp::spin(shared_from_this());
            // rclcpp::shutdown();
            
        }

        void update_behavior_tree() {
            BT::NodeStatus tree_status= tree_.tickOnce();
            while (rclcpp::ok() && tree_status == BT::NodeStatus::RUNNING){
                tree_status=tree_.tickWhileRunning(std::chrono::milliseconds(500)); 
            }

        }
        std::string tree_xml_file_;
        BT::BehaviorTreeFactory factory;
        BT::Tree tree_;
        //std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomyNode>();
    node->execute();
    rclcpp::shutdown();
    return 0;
}