#include "controller/abstract_controller.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
//#include "../include/controller/abstract_controller.h"

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



double abstract_controller::ErrorAngle(turtlesim::Pose cur, geometry_msgs::Pose2D ref)
{
    double Ex = ref.x - cur.x;   //errore lungo x
    double Ey = ref.y - cur.y;   //errore lungo y  
    double ref_theta = atan2f(Ey, Ex);   //stima dell'angolo desiderato
    double Et = ref_theta - cur.theta;   //errore su theta
    return Et;
}



double abstract_controller::ErrorLinear(turtlesim::Pose cur, geometry_msgs::Pose2D ref)
{
    double Ex = ref.x - cur.x;           //errore lungo x
    double Ey = ref.y - cur.y;           //errore lungo y
    double Etx = pow(pow(Ex,2)+pow(Ey,2),0.5);
    return Etx;
}



void abstract_controller::ReadCurrentPosition(const turtlesim::Pose::ConstPtr& msg)
{
    curr = *msg;
}



void abstract_controller::NewReference()
{  
    double l = 3;
    int n = 4;  //numero dei vertici del poligono
    double delta_theta = 2*M_PI/n;
    
    if(reff.theta > 2*M_PI)
        reff.theta -= 2*M_PI;
    
    reff.x += l*cos(reff.theta);
    reff.y += l*sin(reff.theta);
    reff.theta += delta_theta;
}



void abstract_controller::init()
{
    cur_pose_sub = n.subscribe("/turtle1/pose", 1000, &abstract_controller::ReadCurrentPosition, this);        
    comand_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);   // semplice nodo
}



void abstract_controller::run()
{
}