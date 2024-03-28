#include <chrono>
#include <random>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav_behavior.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

const std::string default_bt_xml_file = ament_index_cpp::get_package_share_directory("behavior_tree_demo_ros2");
const std::string xml_file= default_bt_xml_file + "/behavior_tree_xml/demo_behavior_tree.xml";


using namespace BT;

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
  
    auto nh = rclcpp::Node::make_shared("patrolling_node");
    //nh->declare_parameter("bt_xml", rclcpp::ParameterValue(std::string(default_bt_xml_file)));
    //std::string bt_xml;
    //nh->get_parameter("bt_xml", bt_xml);
    //RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", bt_xml.c_str());
  
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;
  
    factory.registerNodeType<Nav2Client>("Nav2Client");
   
  
    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. IMPORTANT: when the
    // object "tree" goes out of scope, all the TreeNodes are destroyed
    Tree tree_ = factory.createTreeFromFile(xml_file);
  
    // Create a logger
    StdCoutLogger logger_cout(tree_);
  
    NodeStatus status = NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while(rclcpp::ok() && status == NodeStatus::RUNNING) 
    {
        status = tree_.tickOnce();
        tree_.sleep(std::chrono::milliseconds(100));
        //status = tree_.tickWhileRunning(std::chrono::milliseconds(500)); 
    }
  
    return 0;
}