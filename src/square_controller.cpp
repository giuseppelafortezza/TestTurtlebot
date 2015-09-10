#include "controller/square_controller.h"
#include <ros/init.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>


void square_controller::config_callback(controller::config_toolConfig &config, uint32_t level) {
  //ROS_INFO("Reconfigure Request: %f %f %f %f", config.kp1, config.kp2, config.ki1, config.ki2);
  kp1 = config.kp1;
  kp2 = config.kp2;
  ki1 = config.ki1;
  ki2 = config.ki2;  
  vertex = config.Polygon;
  rho = 0.4*vertex;
  if(config.CLEAR)
  {
      //rosservice call clear;
    config.CLEAR = 0;
  }
  if(config.RESET)
  {
      kp1 = 0.5;
      kp2 = 7.5;
      ki1 = 0;
      ki2 = 0;
      //rosservice call clear;
      config.kp1 = kp1;
      config.kp2 = kp2;
      config.ki1 = ki1;
      config.ki2 = ki2;   
      config.RESET = 0;
  }     
}



square_controller::square_controller()
{
    NewReference();
}

    
void square_controller::init()
{
    abstract_controller::init();
    f = boost::bind(&square_controller::config_callback, this, _1, _2);   
    server.setCallback(f);
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
        
        if (error_lin < 0.1)
        {
            NewReference();
            error_ang_old = 0;
            error_lin_old = 0;
        }
        
        comand_pub.publish(twist);
        
        
        std_msgs::String mess;
        std::stringstream ss;
        //ss << "\nCurrent: x = " << curr.x  << "   y = " << curr.y << "   theta = " << curr.theta << "\n";
        ss << "Reference: x = " << reff.x  << "   y = " << reff.y << "\n";
        ss << "Error: (" << reff.x-curr.x << " , " << reff.y-curr.y << ")";   
        mess.data = ss.str();
        
        ROS_INFO("%s", mess.data.c_str());
       
        
        
        ros::spinOnce();
        
        loop_rate.sleep();        
    }
}