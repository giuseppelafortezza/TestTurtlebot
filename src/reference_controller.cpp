#include "controller/abstract_controller.h"
#include "controller/reference_controller.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

reference_controller::reference_controller()
{
}



void reference_controller::ReadReferencePosition(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    reff = *msg;
}



void reference_controller::init()
{     
    ref_pose_sub = n.subscribe("/turtle1/reference_pose", 1000, &reference_controller::ReadReferencePosition, this);
    abstract_controller::init();
}



void reference_controller::run()
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
        ss << "\nCurrent: x = " << curr.x  << "   y = " << curr.y << "   theta = " << curr.theta << "\n";
        ss << "Reference: x = " << reff.x  << "   y = " << reff.y << "   theta = " << reff.theta << "\n";
        ss << "Error: (" << reff.x-curr.x << " , " << reff.y-curr.y << ")";   
        mess.data = ss.str();
        ROS_INFO("%s", mess.data.c_str());
        
        ros::spinOnce();
        loop_rate.sleep();        
    }
}