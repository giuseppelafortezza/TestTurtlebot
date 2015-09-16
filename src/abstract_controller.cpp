#include "controller/abstract_controller.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

abstract_controller::abstract_controller()
{
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    
    error_ang = 0;
    error_lin = 0;
    error_ang_old = 0;
    error_lin_old = 0;
}



double abstract_controller::ErrorAngle(nav_msgs::Odometry cur, geometry_msgs::Pose2D ref)
{
    tf::Quaternion q(cur.pose.pose.orientation.x, cur.pose.pose.orientation.y, cur.pose.pose.orientation.z, cur.pose.pose.orientation.w);    
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    
    
    double Ex = ref.x - cur.pose.pose.position.x;   //errore lungo x
    double Ey = ref.y - cur.pose.pose.position.y;   //errore lungo y  
    double ref_theta = atan2f(Ey, Ex);   //stima dell'angolo desiderato
    double cur_theta = yaw;
    reff.theta = ref_theta;
    double Et = ref_theta-cur_theta;   //errore su theta
    return Et;
}



double abstract_controller::ErrorLinear(nav_msgs::Odometry cur, geometry_msgs::Pose2D ref)
{
    double Ex = ref.x - cur.pose.pose.position.x;           //errore lungo x
    double Ey = ref.y - cur.pose.pose.position.y;           //errore lungo y
    double Etx = pow(pow(Ex,2)+pow(Ey,2),0.5);
    return Etx;
}



void abstract_controller::ReadCurrentPosition(const nav_msgs::Odometry::ConstPtr& msg)
{
    curr = *msg;
}



void abstract_controller::NewReference()
{  
    double delta_theta = 2*M_PI/vertex;
    double circ_center_x = 0;
    double circ_center_y = 0;

    reff.x = circ_center_x + rho*cos(alpha);    //new reference point
    reff.y = circ_center_y + rho*sin(alpha);
    
    alpha += delta_theta;
}



void abstract_controller::init()
{
    cur_pose_sub = n.subscribe("odom", 1000, &abstract_controller::ReadCurrentPosition, this);        
    comand_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);  
}



void abstract_controller::run()
{
}