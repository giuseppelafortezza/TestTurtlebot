#ifndef SQUARE_CONTROLLER_H
#define SQUARE_CONTROLLER_H
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>
#include "abstract_controller.h"

class square_controller:public abstract_controller
{    
public:
    square_controller();
    void init();
    void run();
};

#endif //SQUARE_CONTROLLER_H