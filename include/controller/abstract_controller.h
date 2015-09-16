#ifndef ABSTRACT_CONTROLLER_H
#define ABSTRACT_CONTROLLER_H
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

class abstract_controller
{
protected:
    nav_msgs::Odometry curr;
    geometry_msgs::Pose2D reff;
    geometry_msgs::Twist twist;
    double kp1 = 0;
    double kp2 = 0;
    double ki1 = 0;
    double ki2 = 0;
    int vertex = 3; //numero dei vertici del poligono
    double alpha = M_PI/4;
    double rho = 0.7*vertex;     //raggio della circonferenza circoscritta
    ros::NodeHandle n;
    ros::Subscriber cur_pose_sub;
    ros::Publisher comand_pub;
    double error_ang;
    double error_lin;
    double error_ang_old;
    double error_lin_old;
    double typeController = 1;  //0 = auto controller   1 = point controller

protected:
    void ReadCurrentPosition(const nav_msgs::Odometry::ConstPtr& msg);
    double ErrorAngle(nav_msgs::Odometry cur, geometry_msgs::Pose2D ref);
    double ErrorLinear(nav_msgs::Odometry cur, geometry_msgs::Pose2D ref);
    virtual void NewReference();
    
public:
    abstract_controller();
    virtual void init();
    virtual void run();
};

#endif //ABSTRACT_CONTROLLER_H