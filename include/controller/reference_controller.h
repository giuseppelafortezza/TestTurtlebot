#ifndef REFERENCE_CONTROLLER_H
#define REFERENCE_CONTROLLER_H
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>
#include "controller/abstract_controller.h"

class reference_controller:public abstract_controller
{
private: 
    ros::Subscriber ref_pose_sub;
    
private:
    void ReadReferencePosition(const geometry_msgs::Pose2D::ConstPtr& msg);
    
public:
    reference_controller();
    void init();
    void run();
};



#endif //REFERENCE_CONTROLLER_H