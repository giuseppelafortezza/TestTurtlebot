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

  if(config.RESET)
  {
      kp1 = 0;
      kp2 = 0;
      ki1 = 0;
      ki2 = 0;
      //rosservice call clear;
      config.kp1 = kp1;
      config.kp2 = kp2;
      config.ki1 = ki1;
      config.ki2 = ki2;   
      config.RESET = 0;
  }     
  
  if(config.POINT == 1 && typeController == 0)
  {
      typeController = 1;
      config.AUTO = 0;
      config.POINT = 1;
  }
 
  
  if(config.AUTO == 1 && typeController == 1)
  {
      typeController = 0;
      config.POINT = 0;
      config.AUTO = 1;
      NewReference();
  }
  
  if(config.Apply == 1 && typeController == 1)
  {
      reff.x = config.x;
      reff.y = config.y;
      config.Apply = 0;
  }
  
}



square_controller::square_controller()
{
    reff.x = 0;
    reff.y = 0;
    reff.theta = 0;
}

    
void square_controller::init()
{
    abstract_controller::init();
    f = boost::bind(&square_controller::config_callback, this, _1, _2);   
    server.setCallback(f);
}



void square_controller::run()
{
    ros::Rate loop_rate(20);
    
    const double MAX_TWIST_LINEAR = 0.5;
    const double MAX_TWIST_ANGULAR = 0.5;
    
    while (ros::ok())
    {        
        error_lin = ErrorLinear(curr,reff);
        error_ang = ErrorAngle(curr,reff);        
        error_lin_old += error_lin;
        error_ang_old += error_ang;
        
        
        
        twist.angular.z = kp2*sin(error_ang);
        twist.linear.x = kp1*error_lin;     
        
        // Saturazione sul twist comandato
        if(twist.linear.x > MAX_TWIST_LINEAR)
            twist.linear.x = MAX_TWIST_LINEAR;
        
        if(twist.angular.z > MAX_TWIST_ANGULAR)
            twist.angular.z = MAX_TWIST_ANGULAR;
        
        
        comand_pub.publish(twist);
        
        if(error_lin < 0.1)
        {
            twist.angular.z = 0;
            
            if(error_lin < 0.03)
                twist.linear.x = 0;
            
            if(typeController == 0)
                NewReference();
        }
        
        std_msgs::String mess;
        std::stringstream ss;
        ss << "\nCurrent: x = " << curr.pose.pose.position.x  << "   y = " << curr.pose.pose.position.y << "   theta = " << curr.pose.pose.orientation.z*180/M_PI << "\n";
        ss << "Reference: x = " << reff.x  << "   y = " << reff.y <<  "  theta = " << reff.theta*180/M_PI << "\n";
        ss << "Linear error: " << pow(pow(reff.x-curr.pose.pose.position.x,2)+pow(reff.y-curr.pose.pose.position.y,2),0.5) << " m. \n";
        ss << "Angular error: " << (reff.theta-curr.pose.pose.orientation.z)*180/M_PI << " deg. \n";
        ss << "Twist: \n" << twist.linear.x << "\n" << twist.linear.y << "\n" << twist.linear.z << "\n" << twist.angular.x << "\n" << twist.angular.y << "\n" << twist.angular.z << " \n";
        ss << "Type controller: ";
        if(typeController)
            ss<< "Point controller" << "\n";
        else
            ss<< "Auto controller" << "\n";
        
        mess.data = ss.str();
        
        ROS_INFO("%s", mess.data.c_str());
       
        
        
        ros::spinOnce();
        
        loop_rate.sleep();        
    }
}