#include "controller/controller.h"
#include <ros/ros.h>

controller::controller()
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



double controller::ErrorAngle(turtlesim::Pose cur, geometry_msgs::Pose2D ref)
{
    double Ex = ref.x - cur.x;   //errore lungo x
    double Ey = ref.y - cur.y;   //errore lungo y  
    double ref_theta = atan2f(Ey, Ex);   //stima dell'angolo desiderato
    double Et = ref_theta - cur.theta;   //errore su theta
    return Et;
}



double controller::ErrorLinear(turtlesim::Pose cur, geometry_msgs::Pose2D ref)
{
    double Ex = ref.x - cur.x;           //errore lungo x
    double Ey = ref.y - cur.y;           //errore lungo y
    double Etx = pow(pow(Ex,2)+pow(Ey,2),0.5);
    return Etx;
}



void controller::ReadCurrentPosition(const turtlesim::Pose_< std::allocator >::ConstPtr& msg)
{
    curr = *msg;
}



void controller::ReadReferencePosition(const geometry_msgs::Pose2D_< std::allocator >::ConstPtr& msg)
{
    reff = *msg;
}



void controller::init()
{
    ros::NodeHandle n;
    cur_pose_sub = n.subscribe("/turtle1/pose", 1000, &controller::ReadCurrentPosition, &curr);        
    ref_pose_sub = n.subscribe("/turtle1/reference_pose", 1000, &controller::ReadReferencePosition, &reff);
    comand_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);   // semplice nodo
}



void controller::run()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {        
        error_lin = ErrorLinear(curr,reff);
        error_ang = ErrorAngle(curr,reff);
        error_lin_old += error_lin;
        error_ang_old += error_ang;
        
        if(error_lin > 0)
        {
            twist.linear.x = kp1*error_lin + ki1*error_lin_old;
            twist.angular.z = kp2*sin(error_ang) + ki2*error_ang_old;
        }
        else
        {
            twist.linear.x = 0;
            twist.angular.z = 0;
        }
        
        comand_pub.publish(twist);
        
        
        std_msgs::String mess;
        std::stringstream ss;
        ss << "\nCurrent: x = " << cur.x  << "   y = " << cur.y << "   theta = " << cur.theta << "\n";
        ss << "Reference: x = " << ref.x  << "   y = " << ref.y << "   theta = " << ref.theta << "\n";
        ss << "Error: (" << ref.x-cur.x << " , " << ref.y-cur.y << ")";   
        mess.data = ss.str();
        ROS_INFO("%s", mess.data.c_str());
        
        ros::spinOnce();
        loop_rate.sleep();        
    }
}