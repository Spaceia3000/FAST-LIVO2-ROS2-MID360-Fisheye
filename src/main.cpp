#include "LIVMapper.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  {
    LIVMapper mapper("laserMapping", options);
    mapper.initializeSubscribersAndPublishers();
    mapper.run();
  }
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
  return 0;
}
