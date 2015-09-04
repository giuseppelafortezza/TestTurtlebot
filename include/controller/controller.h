#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>

class controller
{
private:
    turtlesim::Pose curr;
    geometry_msgs::Pose2D reff;
    geometry_msgs::Twist twist; 
    double kp1 = 1.5;
    double kp2 = 5;
    double ki1 = 0;
    double ki2 = 0;
    ros::Subscriber cur_pose_sub;
    ros::Subscriber ref_pose_sub;
    ros::Publisher comand_pub;
    double error_ang;
    double error_lin;
    double error_ang_old;
    double error_lin_old;
    
public:
    controller();
    void ReadCurrentPosition(const turtlesim::Pose::ConstPtr& msg);
    void ReadReferencePosition(const geometry_msgs::Pose2D::ConstPtr& msg);
    double ErrorAngle(turtlesim::Pose cur, geometry_msgs::Pose2D ref);
    double ErrorLinear(turtlesim::Pose cur, geometry_msgs::Pose2D ref);
    void init();
    void run();
};



#endif //CONTROLLER_H