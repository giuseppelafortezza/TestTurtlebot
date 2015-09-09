#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <controller/config_toolConfig.h>
#include <boost/function.hpp>

void callback(controller::config_toolConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f", 
	    config.kp1, config.kp2, config.ki1, config.ki2);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller");

  dynamic_reconfigure::Server<controller::config_toolConfig> server;
  dynamic_reconfigure::Server<controller::config_toolConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
