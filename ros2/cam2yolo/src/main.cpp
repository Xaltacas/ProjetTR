

#include <iostream>

#include "cam2yolo.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<Cam2yolo>(options);
  //RCLCPP_INFO(node->get_logger(),"Help me Obi-Wan Kenobi, you're my only hope");
  rclcpp::spin(node);
  rclcpp::shutdown();
  std::cout<< "end" << std::endl;
  return 0;
}

