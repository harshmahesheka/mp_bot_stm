#include <ros.h>
#include <std_msgs/UInt16.h>
#include<geometry_msgs/Point.h>



#define wheelradius 0.035
#define TPR 1
#define wheeltrack 0.28


geometry_msgs::Point odom;

double l_last_tick=0;
double r_last_tick=0;
double radius=0;
double dl=0;
double dr=0;
double dc=0;
double dth=0; 
double th=0;
double dx=0;
double dy=0;
double dt=0;
double iccX=0;
double iccY=0;
double odom_x=0;
double odom_y=0;
double odom_th=0;
double pos=0;
long inter_val=0;
long unsigned int prev_time;
long cnt;
double l_vel=0;
double r_vel=0;
double x_vel=0;
double th_vel=0;
double l_wheel_vel=0;
double r_wheel_vel=0;
double last_x;
double last_y;
double last_th;
