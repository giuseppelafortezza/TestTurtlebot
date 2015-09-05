#include "controller/square_controller.h"
#include <ros/init.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>

square_controller::square_controller()
{
    reff.x = 4;
    reff.y = 4;
    reff.theta = 0;
}

    
void square_controller::init()
{
    abstract_controller::init();
}



void square_controller::run()
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
            twist.angular.z = kp2*sin(error_ang) + ki2*error_ang_old;
        }
        
        if (error_lin < 0.001 && fabs(error_ang) < 0.5)
            NewReference();
        
        comand_pub.publish(twist);
        
        
        std_msgs::String mess;
        std::stringstream ss;
        ss << "\nCurrent: x = " << curr.x  << "   y = " << curr.y << "   theta = " << curr.theta << "\n";
        ss << "Reference: x = " << reff.x  << "   y = " << reff.y << "   theta = " << reff.theta << "\n";
        ss << "Error: (" << reff.x-curr.x << " , " << reff.y-curr.y << ")";   
        mess.data = ss.str();
        ROS_INFO("%s", mess.data.c_str());
        
        ros::spinOnce();
        loop_rate.sleep();        
    }
}