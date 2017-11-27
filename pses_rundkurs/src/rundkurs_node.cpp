#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include<iostream>
#include<iomanip>
#include <math.h>

using namespace std;

void laserCallback(sensor_msgs::LaserScan::ConstPtr laserMsg, sensor_msgs::LaserScan* laser)
{
  *laser = *laserMsg;
}

void motor0Callback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* retu)
{
  *retu = *msg;
}

void motor1Callback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* retu)
{
  *retu = *msg;
}

void steering0Callback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* retu)
{
  *retu = *msg;
}

void steering1Callback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* retu)
{
  *retu = *msg;
}

void cornerCallback(std_msgs::Bool::ConstPtr msg, std_msgs::Bool* retu)
{
  *retu = *msg;
}

//returns true if first 10 not nan numbers of laser.ranges are greater than 3
bool corner(sensor_msgs::LaserScan laser)
{
    if(laser.ranges.empty())
        return false;

  // ROS_INFO("%f", laser.range_max);
  // ROS_INFO("%f", laser.ranges[6]);
     int i=3;
    int j=0;
    while(j<=8){
        if(isnan(laser.ranges[i]))
            i++;
        else{
            if(laser.ranges[i]<3.2)
                return false;
            i++;
            j++;
        }
    }


    return true;
}

//returns true if corner_type is long
bool long_corner(sensor_msgs::LaserScan laser)
{
    //int h=253; ist etwa Mitte
    int h=190;
   while(isnan(laser.ranges[h]))
       h++;
    if(laser.ranges[h]<3.2)
        return false;
    return true;
}

int main(int argc, char** argv)
{
    // init this node
    ros::init(argc, argv, "rundkurs_node");
    // get ros node handle
    ros::NodeHandle nh;
    // sensor message container
    sensor_msgs::LaserScan laser;
    std_msgs::Int16 motor0in;
    std_msgs::Int16 motor1in;
    std_msgs::Int16 steering0in;
    std_msgs::Int16 steering1in;
    std_msgs::Bool corner_fin;
    std_msgs::Bool corner_start;
    std_msgs::Bool corner_type;
    std_msgs::Bool hallway_type;
    //debug
    std_msgs::Int16 debug;
    debug.data =0;
    // generate subscriber for sensor messages
    ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>( "/scan", 1, boost::bind(laserCallback, _1, &laser));
    ros::Subscriber motor0Sub = nh.subscribe<std_msgs::Int16>( "wall_motor_level", 1, boost::bind(motor0Callback, _1, &motor0in));
    ros::Subscriber motor1Sub = nh.subscribe<std_msgs::Int16>( "corner_motor_level", 1, boost::bind(motor1Callback, _1, &motor1in));
    ros::Subscriber steering0Sub = nh.subscribe<std_msgs::Int16>( "wall_steering_level", 1, boost::bind(steering0Callback, _1, &steering0in));
    ros::Subscriber steering1Sub = nh.subscribe<std_msgs::Int16>( "corner_steering_level", 1, boost::bind(steering1Callback, _1, &steering1in));
    ros::Subscriber cornerSub = nh.subscribe<std_msgs::Bool>( "simple_corner_finish_inf", 1, boost::bind(cornerCallback, _1, &corner_fin));

    //generate publisher
    ros::Publisher motorCtrl =
        nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
    ros::Publisher steeringCtrl =
        nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);
    ros::Publisher cornerStart =
        nh.advertise<std_msgs::Bool>("begin_curve", 1);
    ros::Publisher cornerType =
        nh.advertise<std_msgs::Bool>("corner_type", 1);
    ros::Publisher hallwayType =
        nh.advertise<std_msgs::Bool>("hallway_type", 1);

    //wait for Kinect
    ros::Duration(1).sleep();
    //want some Data pls
    ros::spinOnce();

    int corner_count = 0;

    //state ist 0 für gerade aus, in Kurven 1
    int state;

    corner_start.data = true;
    cornerStart.publish(corner_start);



   //if start is at corner
    if(corner(laser)){
        state=1;
        corner_start.data = false;
        cornerStart.publish(corner_start);
        corner_type.data=long_corner(laser);
        hallway_type.data=corner_type.data;
        ROS_INFO("Ecke: %d", corner_type.data);
        cornerType.publish(corner_type);
        hallwayType.publish(hallway_type);
        corner_count++;
        state=1;
        ROS_INFO("corner detected");
    }
    else{
        state=0;
        float sum=laser.ranges[3]+laser.ranges[250];
        ROS_INFO("Range: %f", sum);
        //if car starts in small hallway
        if(sum<4.0)
            corner_type.data=0;
        else
            corner_type.data=1;
        ROS_INFO("Gang: %d", corner_type.data);
        hallway_type.data=corner_type.data;
        cornerType.publish(corner_type);
        hallwayType.publish(hallway_type);
    }


     ros::Rate loop_rate(25);

    while (ros::ok()){
       // ROS_INFO("rundkurs running");

       // if(corner_count>4)
       //     break;
        switch(state){
        case 0:{
            motorCtrl.publish(motor0in);
            steeringCtrl.publish(steering0in);
            corner_start.data = true;
            cornerStart.publish(corner_start);
            if(corner(laser)){
                corner_start.data = false;
                cornerStart.publish(corner_start);
                corner_type.data=long_corner(laser);
                hallway_type.data=corner_type.data;
                ROS_INFO("Ecke: %d", corner_type.data);
                cornerType.publish(corner_type);
                hallwayType.publish(hallway_type);
                corner_count++;
                state=1;
                ROS_INFO("corner detected");
            }
            break;
            }
            case 1:{
                //ROS_INFO("%d", steering1in.data);
                motorCtrl.publish(motor1in);
                steeringCtrl.publish(steering1in);
                if(corner_fin.data){
                    state=0;
                    ROS_INFO("corner finished________________");
                 }
                break;
            }
        case 42:{
            motorCtrl.publish(debug);
            break;
        }
        }



   loop_rate.sleep();
   ros::spinOnce();
    }
    ros::spin();
return 0;
}
