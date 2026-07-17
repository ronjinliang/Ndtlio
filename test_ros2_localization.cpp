#include "ros2node/ndt_lio_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationNode>("ndt_lio_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}






