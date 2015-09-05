#ifndef ABSTRACT_CONTROLLER_H
#define ABSTRACT_CONTROLLER_H
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>

class abstract_controller
{
protected:
    turtlesim::Pose curr;
    geometry_msgs::Pose2D reff;
    geometry_msgs::Twist twist; 
    double kp1 = 1;
    double kp2 = 7.5;
    double ki1 = 0;
    double ki2 = 0;
    ros::NodeHandle n;
    ros::Subscriber cur_pose_sub;
    ros::Publisher comand_pub;
    double error_ang;
    double error_lin;
    double error_ang_old;
    double error_lin_old;

protected:
    void ReadCurrentPosition(const turtlesim::Pose::ConstPtr& msg);
    double ErrorAngle(turtlesim::Pose cur, geometry_msgs::Pose2D ref);
    double ErrorLinear(turtlesim::Pose cur, geometry_msgs::Pose2D ref);
    virtual void NewReference();
    
public:
    abstract_controller();
    virtual void init();
    virtual void run();
};

#endif //ABSTRACT_CONTROLLER_H