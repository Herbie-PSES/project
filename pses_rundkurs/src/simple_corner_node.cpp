#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <ctime>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <math.h>

#include"MiniPID.h"


void odomCallback(nav_msgs::Odometry::ConstPtr odomMsg, nav_msgs::Odometry* odo)
{
  *odo = *odomMsg;
}

void begin_cur_Callback(std_msgs::Bool::ConstPtr beginMsg, std_msgs::Bool* begin)
{
  *begin = *beginMsg;
}

void cur_type_Callback(std_msgs::Bool::ConstPtr typeMsg, std_msgs::Bool* type)
{
  *type = *typeMsg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_corner_node");
    ros::NodeHandle nh;




      nav_msgs::Odometry odo;
      std_msgs::Int16 motor, steering;
      std_msgs::Bool curve_finished;
      std_msgs::Bool begin;
      std_msgs::Bool corner_type;

      ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>(
                  "/odom",10,boost::bind(odomCallback,_1,&odo));
      ros::Subscriber begin_cur =nh.subscribe<std_msgs::Bool>("begin_curve",10,boost::bind(begin_cur_Callback,_1,&begin));
      ros::Subscriber cur_type =nh.subscribe<std_msgs::Bool>("begin_curve",10,boost::bind(cur_type_Callback,_1,&corner_type));

      // generate control message publisher
      ros::Publisher motorCtrl =
          nh.advertise<std_msgs::Int16>("corner_motor_level", 1);
      ros::Publisher steeringCtrl =
          nh.advertise<std_msgs::Int16>("corner_steering_level", 1);
      ros::Publisher curve_finished_inf = nh.advertise<std_msgs::Bool>("simple_corner_finish_inf",1);

      // Loop starts here:
      // loop rate value is set in Hz
      ros::Rate loop_rate(25);

    enum turn{l,r};

	
	// set direction,angle,speed,
    turn direction=r;
    double angle = 90;
    int speed=350;
    int steering_angle =225;
    int corner_mode=0;
    double roll, pitch, yaw,degree,old_degree,solldiff,set_angle;
    double set_back_360 =0;
	
        //angle and steering_angle right/left adjustment
    if(direction==r){
     angle=-angle;
     steering_angle=-steering_angle;

    }

        //test
    curve_finished.data = false;


    //  pid.setOutputLimits(-lim,lim);
      while (ros::ok())
      {


          tf::Quaternion q;
          tf::quaternionMsgToTF(odo.pose.pose.orientation, q);
          tf::Matrix3x3 mat(q);
          mat.getEulerYPR(yaw, pitch, roll);
           degree =(yaw*180/M_PI)+180;
       /*    if(degree<0){
               degree=0;
           }*/

        curve_finished.data = false;

       //set start orientation
       if(begin.data&& degree>=0){
           set_angle=degree;
           set_back_360=0;
        }

       if(corner_type.data){
           steering_angle =140;
           speed=500;
       }
       else{
           steering_angle =222;
           speed=400;
       }

//

       //360 to 0 jump degree
       if(old_degree-degree>=300){
        set_back_360=360;
        //degree=degree+360;
       }

       if(old_degree-degree<=-300){
           set_back_360 = -360;
          // degree=degree-360;
       }

		//degree difference
        solldiff=degree-set_angle-angle+set_back_360;

		//turn
        if(abs(solldiff)>=5){
          steering.data = steering_angle;
           motor.data=speed;
        }else{
            curve_finished.data = true;
        }
		
		//set old_degree for 360 to 0 jump
        old_degree=degree;


        // publish command messages on their topics
        motorCtrl.publish(motor);
        steeringCtrl.publish(steering);
        curve_finished_inf.publish(curve_finished);
        // side note: setting steering and motor even though nothing might have
        // changed is actually stupid but for this demo it doesn't matter too much.

        // clear input/output buffers
        ros::spinOnce();
        // this is needed to ensure a const. loop rate
        loop_rate.sleep();
      }

      ros::spin();
    }

