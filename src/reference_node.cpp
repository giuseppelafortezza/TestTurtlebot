#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include <sstream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reference_node");
    ros::NodeHandle n;

    ros::Publisher reference_pub = n.advertise<geometry_msgs::Pose2D>("/turtle1/reference_pose", 1000);

    ros::Rate loop_rate(0.1);
   
    
    int count = 0;
    float r = 1;
    std::vector<double> q_x = {2, 7, 7, 2};
    std::vector<double> q_y = {2, 2, 7, 7};
    
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        geometry_msgs::Pose2D ref_pose;
        
       
        //int q = count % 360;
        //ref_pose.x = 5.5 + r*cos(2*M_PI*q/360);
        //ref_pose.y = 5.5 + r*sin(2*M_PI*q/360); 
        ref_pose.x = q_x[count % 4];
        ref_pose.y = q_y[count % 4];
        
        
        ss << "x = " << ref_pose.x << "   y = " << ref_pose.y;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());


        reference_pub.publish(ref_pose);
        
        r = r + 0.001;
        
        if (count == 360)
            count = 0;
        else
            count++;
        
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}