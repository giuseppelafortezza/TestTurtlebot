#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"                                // to read current position
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Pose2D.h>
#include <sstream>
#include <math.h>

class Current_pose
{
public:
    double x;
    double y;
    double theta;
    turtlesim::Pose curr;
     
    Current_pose()
    {
        x = 0;
        y = 0;
        theta = 0;
        curr.x = 0;
        curr.y = 0;
        curr.theta = 0;
    }
        
    void ReadCurrentPosition(const turtlesim::Pose::ConstPtr& msg)
    {
        x = msg->x;
        y = msg->y;
        theta = msg->theta;
        curr.x = msg->x;
        curr.y = msg->y;
        curr.theta = msg->theta;
    }
};


class Reference_pose
{  
public:
    double x;
    double y;
    double theta;
    geometry_msgs::Pose2D reff;
    
    Reference_pose()
    {
        x = 0;
        y = 0;
        theta = 0;
        reff.x = 0;
        reff.y = 0;
        reff.theta = 0;
        
    }
    
    void ReadReferencePosition(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        x = msg->x;
        y = msg->y;
        theta = msg->theta;
        reff.x = msg->x;
        reff.y = msg->y;
        reff.theta = msg->theta;
    }
};


double ErrorAngle(turtlesim::Pose cur, geometry_msgs::Pose2D ref)
{
    double Ex = ref.x - cur.x;   //errore lungo x
    double Ey = ref.y - cur.y;   //errore lungo y
    
    double ref_theta = atan2f(Ey, Ex);   //stima dell'angolo desiderato
    
    if (fabs(ref_theta) < 0.000002)
        std::cout << ref_theta << std::endl;
    double Et = ref_theta - cur.theta;   //errore su theta
    
    return Et;
}

double ErrorLinear(turtlesim::Pose cur, geometry_msgs::Pose2D ref)
{
    double Ex = ref.x - cur.x;           //errore lungo x
    double Ey = ref.y - cur.y;           //errore lungo y
    
    double Etx = pow(pow(Ex,2)+pow(Ey,2),0.5);
    
    return Etx;
}
    
    
using namespace std;

int main(int argc, char **argv)
{
    cout << atan2f(1,1) <<  "  " << M_PI/4 << endl;
    cout << atan2f(1,-1) <<  "  " << 5*M_PI/4 << endl;
    cout << atan2f(-1,-1) <<  "  " << -3*M_PI/4 << endl;
    cout << atan2f(1,0) <<  "  " << M_PI/2 << endl;
    cout << atan2f(-1,0) <<  "  " << -M_PI/2 << endl;
    cout << atan2f(0,0) <<  " Nan " << endl;
    
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

        
    Reference_pose ref;
    Current_pose cur;
    geometry_msgs::Twist twist; 
    
    
    ros::Subscriber cur_pose_sub = n.subscribe("/turtle1/pose", 1000, &Current_pose::ReadCurrentPosition, &cur);        
    ros::Subscriber ref_pose_sub = n.subscribe("/turtle1/reference_pose", 1000, &Reference_pose::ReadReferencePosition, &ref);
    ros::Publisher comand_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);   // semplice nodo
      
    // variabili non utilizzate poichè nel movimento planare viene utilizzato solo l'avanzamento su x e la rotazione su z
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    
    // parametri controllori
    double kp1 = 1.5;
    double kp2 = 5;
    double ki1 = 0;
    double ki2 = 0;
    
    // inizializzazione errori
    double error_ang = 0;
    double error_lin = 0;
    double error_ang_old = 0;
    double error_lin_old = 0;
    
    while (ros::ok())
    {        
        error_lin = ErrorLinear(cur.curr,ref.reff);
        error_ang = ErrorAngle(cur.curr,ref.reff);
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

    return 0;
}