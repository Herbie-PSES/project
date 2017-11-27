#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <tf/tf.h>

#include "MiniPID.h"


// gets called whenever a new message is availible in the input puffer
void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl)
{
  *usl = *uslMsg;
}

// gets called whenever a new message is availible in the input puffer
void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf)
{
  *usf = *usfMsg;
}

// gets called whenever a new message is availible in the input puffer
void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr)
{
  *usr = *usrMsg;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "wallfollow_node");
  ros::NodeHandle nh;

  // sensor message container
  sensor_msgs::Range usr, usf, usl;
  std_msgs::Int16 motor, steering;

  // generate subscriber for sensor messages
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &usr));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usl", 10, boost::bind(uslCallback, _1, &usl));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usf", 10, boost::bind(usfCallback, _1, &usf));

  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("wall_motor_level", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("wall_steering_level", 1);

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(25);

  enum side{l,r};
  double steering_reg,solldif;


//distance, speed and max steering angle
double distance=0.75;
int speed=500;
double max_steering=550;

//direction
side s =r;

//init controller MiniPID(P,I,D)
MiniPID pid = MiniPID(450,0.1,27000);

//max steering level
pid.setOutputLimits(-max_steering,max_steering);


  while (ros::ok())
  {
	//solldif right/left adjustment
      if(s==l){
          solldif = distance - usl.range;
      }
      else{
          solldif=distance - usr.range;
      }
	  
	// get steering regulation
      steering_reg=pid.getOutput(solldif,0);


	//wallfollow regulation
        if(solldif!=0){
            if(s==l){
              steering.data = -steering_reg;
            }
            else{
              steering.data = steering_reg;
            }
        }

      motor.data = speed;

  /*  else
    {
      steering.data = 0;
      motor.data = 0;
    }
    if (usf.range < 0.15)
    {
      motor.data = 0;
      steering.data = 0;
    }
*/
  //  ROS_INFO("wallfollow running");

    // publish command messages on their topics
    motorCtrl.publish(motor);
    steeringCtrl.publish(steering);

    // side note: setting steering and motor even though nothing might have
    // changed is actually stupid but for this demo it doesn't matter too much.

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
