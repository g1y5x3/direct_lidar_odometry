/************************************************************
 *
 * Copyright (c) 2025, Missouri S&T, Rolla, MO
 *
 * Authors: Yixiang Gao
 * Contact: ygao@mst.edu
 *
 ***********************************************************/
#include "dlo/localization.h"


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<dlo::LocalizationNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  
  node->start();
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  
  return 0;
}