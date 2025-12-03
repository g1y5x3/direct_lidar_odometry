#include "dlo/map_server.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto map_server = std::make_shared<dlo::MapServer>();
  map_server->start();
  rclcpp::spin(map_server);
  rclcpp::shutdown();
  return 0;
}
