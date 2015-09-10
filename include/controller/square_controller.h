#ifndef SQUARE_CONTROLLER_H
#define SQUARE_CONTROLLER_H
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>
#include "abstract_controller.h"

#include <controller/config_toolConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>



class square_controller:public abstract_controller
{    
private:
    dynamic_reconfigure::Server<controller::config_toolConfig> server;
    dynamic_reconfigure::Server<controller::config_toolConfig>::CallbackType f;
public:
    square_controller();
    void config_callback(controller::config_toolConfig &config, uint32_t level);   //NUOVE LINEE
    void init();
    void run();
};

#endif //SQUARE_CONTROLLER_H