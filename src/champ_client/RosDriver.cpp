/***************************************************************************
 
    file                 : RosDriver.cpp
    copyright            : (C) 2011 Sean Payne 
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include "RosDriver.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "sigevo_champ/CarState.h"
#include "sigevo_champ/CarControl.h"
#include <cmath>
#include <signal.h>


#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))
#define RAD2DEG(r) ((r)*(180.0/M_PI))

static const double BROADCAST_LASER_TF_FREQ = 10.0;
static const double BROADCAST_ODOM_TF_FREQ = 10.0;

/////////////////////////////////////////////////////////////////////////////
// RosDriver::RosDriver
//
//  Constructor.  Initializes that which can be initialized.  
//
//  Though most work intitialization for ROS needs argc and argv, so that is
//  done in init0 below.
/////////////////////////////////////////////////////////////////////////////
RosDriver::RosDriver()
{
	p_nodehandle_ = 0;
	p_car_state_publisher_ = 0; 
	p_laser_publisher_ = 0; 
	p_odom_publisher_ = 0; 
	p_car_control_subscriber_= 0; 
	p_tf_broadcaster_= 0; 
	track_sensor_angle_min_ = -M_PI/2.0; 
	track_sensor_angle_max_ = M_PI/2.0; 
	first_odom_calced_ = false;
	odom_x_ = 0.0;
	odom_y_ = 0.0;
 	odom_th_ = 0.0;
	cur_car_control_.setBrake(1.0);	// default state is full brake to stop rolling
}

/////////////////////////////////////////////////////////////////////////////
// RosDriver::init0
//
//  Performs the ROS initialization, sets up the publishers and subscribers
//
/////////////////////////////////////////////////////////////////////////////
void RosDriver::init0(int argc, char **argv)
{
	ros::init(argc, argv, "RosClient");

	p_nodehandle_ = new ros::NodeHandle;
	p_car_state_publisher_ = new ros::Publisher;
	*p_car_state_publisher_ = p_nodehandle_->advertise<sigevo_champ::CarState>("torcs_car_state", 50);

	p_laser_publisher_ = new ros::Publisher;
	*p_laser_publisher_ = p_nodehandle_->advertise<sensor_msgs::LaserScan>("scan", 50);

	p_odom_publisher_ = new ros::Publisher;
	*p_odom_publisher_ = p_nodehandle_->advertise<nav_msgs::Odometry>("odom", 50);

	p_car_control_subscriber_ = new ros::Subscriber;
	*p_car_control_subscriber_ = p_nodehandle_->subscribe("torcs_car_control", 5, &RosDriver::carControlCallback, this);

	p_tf_broadcaster_ = new tf::TransformBroadcaster; 
}

/////////////////////////////////////////////////////////////////////////////
// RosDriver::init
//
// This is a virtual method - it is invoked when the main loop in client.cpp
// wants a list of track angle sensors
/////////////////////////////////////////////////////////////////////////////
void RosDriver::init(float *angles)
{
	ros::NodeHandle nh("~");
	nh.getParam("track_sensor_angle_min", track_sensor_angle_min_);
	nh.getParam("track_sensor_angle_max", track_sensor_angle_max_);

	double angle = track_sensor_angle_min_;
	double increment = (track_sensor_angle_max_ - track_sensor_angle_min_)/TRACK_SENSORS_NUM; 
	for (int i = 0; i < TRACK_SENSORS_NUM; i++)
	{
		angles[i] = RAD2DEG(angle);
		angle += increment;	
	}

   // force periodic things to do their first update on the next call to wDrive 
	time_for_next_laser_tf_broadcast_ = ros::Time::now();
	time_for_next_odom_tf_broadcast_ = ros::Time::now();
	last_car_state_msg_time_ = ros::Time::now();
}

/////////////////////////////////////////////////////////////////////////////
// RosDriver::~RosDriver
//
// Delete all ros objects 
/////////////////////////////////////////////////////////////////////////////
RosDriver::~RosDriver()
{
	delete p_nodehandle_;
	delete p_car_state_publisher_;
	delete p_laser_publisher_;
	delete p_odom_publisher_;
	delete p_car_control_subscriber_;
	delete p_tf_broadcaster_;
}

/////////////////////////////////////////////////////////////////////////////
// msgToCarControl 
//
// sigevo_champ::CarControl is the ROS equivalent to a CarControl class
// that comes with the SIGEVO client software.  This routine just
// converts from the Ros message to the regular "CarControl" class
/////////////////////////////////////////////////////////////////////////////
static void msgToCarControl(const sigevo_champ::CarControl::ConstPtr &msg, CarControl *cc)
{
	*cc = CarControl(msg->accel, msg->brake, msg->gear, msg->steer, msg->clutch, msg->focus, msg->meta);
}

/////////////////////////////////////////////////////////////////////////////
// RosDriver::carControlCallback
//
// when a car control message is received, it is converted to a CarControl
// object and that object is kept as cur_car_control_
//
// Then any time the wDrive is invoked (which wants a CarControl as reponse) 
// this cur_car_control_ is returned 
/////////////////////////////////////////////////////////////////////////////
void RosDriver::carControlCallback(const sigevo_champ::CarControl::ConstPtr &msg) 
{
	msgToCarControl(msg, &cur_car_control_);
}

/////////////////////////////////////////////////////////////////////////////
// carStateToMsg
//
// This function converts from "CarState" which is the name of the C++ class that
// comes with the SIGEVO client to the sigevo_champ::CarState class which is
// the ROS equivalent. 
/////////////////////////////////////////////////////////////////////////////
static void carStateToMsg(CarState &cs, sigevo_champ::CarState &msg)
{
	msg.angle = cs.getAngle();
	msg.curLapTime = cs.getCurLapTime();
	msg.damage = cs.getDamage();
	msg.distFromStart = cs.getDistFromStart();
	msg.distRaced = cs.getDistRaced();
	for (int i = 0; i < FOCUS_SENSORS_NUM; i++) 
	{
		msg.focus[i] =  cs.getFocus(i);
	}
	msg.fuel = cs.getFuel();
	msg.gear = cs.getGear();
	msg.lastLapTime = cs.getLastLapTime();	
	for (int i = 0; i < OPPONENTS_SENSORS_NUM; i++) 
	{
		msg.opponents[i] =  cs.getOpponents(i);
	}
	msg.racePos = cs.getRacePos();
	msg.rpm = cs.getRpm();
	msg.speedTH = cs.getSpeedTH();
	msg.speedX = cs.getSpeedX();
	msg.speedY = cs.getSpeedY();
	msg.speedZ = cs.getSpeedZ();
	for (int i = 0; i < TRACK_SENSORS_NUM; i++)
	{
		msg.track[i] = cs.getTrack(i);
	}
	msg.trackPos = cs.getTrackPos();
	for (int i = 0; i < 4; i++)
	{
		msg.wheelSpinVel[i] = cs.getWheelSpinVel(i);
	}
	msg.z = cs.getZ();
}

/////////////////////////////////////////////////////////////////////////////
// carStateToLaserScan
//
// yet another conversion routine: it takes CarState messages and using
// the information contained in cs.GetTrack() (the track sensor values)
// converts them to the ROS equivalent laser scan messages. 
/////////////////////////////////////////////////////////////////////////////
static void carStateToLaserScan(CarState &cs, ros::Time scan_time, double time_increment, double track_sensor_angle_min, double track_sensor_angle_max, sensor_msgs::LaserScan *scan)
{
	scan->header.stamp = scan_time;
	scan->header.frame_id = "/base_laser";
	scan->angle_min =  track_sensor_angle_min;
	scan->angle_max =  track_sensor_angle_max;
	scan->angle_increment =  (track_sensor_angle_max - track_sensor_angle_min)/TRACK_SENSORS_NUM; 
	scan->time_increment = 0.0; 
	scan->scan_time = 0.0; 
	scan->range_min = 0.0; 
	scan->range_max = 200.0; 
	scan->ranges.resize(TRACK_SENSORS_NUM);

	for (int i = 0; i < TRACK_SENSORS_NUM; i++)
	{
		scan->ranges[TRACK_SENSORS_NUM - i - 1] = cs.getTrack(i); 
	}
}

/////////////////////////////////////////////////////////////////////////////
// publishLaserTF 
//
// publishes the transform from the base_link to the laser itself.
// In this case they are the same (origin and rotation are 0 and identity)
/////////////////////////////////////////////////////////////////////////////
void RosDriver::publishLaserTF()
{
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0,0,0) );
	transform.setRotation( tf::createIdentityQuaternion() );
 	p_tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/base_laser"));
}

/////////////////////////////////////////////////////////////////////////////
// recalculateOdomValues
//
// Using car state velocities and the time ince the last update,
// extrapolate some new odometry values 
//
// This code is based on:  
//   http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom 
/////////////////////////////////////////////////////////////////////////////
void RosDriver::recalculateOdomValues(CarState &cs)
{
	ros::Time current_time = ros::Time::now();

	// if it's the first time - initialize the 'last values' with the
	// current values
	if (!first_odom_calced_)
	{
		last_odom_calc_dist_ = cs.getDistRaced(); 
		last_odom_calc_time_ = current_time;
		first_odom_calced_ = true; 
	}

	//compute odometry in a typical way given the velocities of the robot
	// convert from km/hr to m/s
	double vx = cs.getSpeedX() * 1000.0/3600.0;
	double vy = cs.getSpeedY() * 1000.0/3600.0;
	double vth = cs.getSpeedTH() * 1000.0/3600.0;

	double dt = (current_time - last_odom_calc_time_).toSec();

	double delta_x = (vx * cos(odom_th_) - vy * sin(odom_th_)) * dt;
	double delta_y = (vx * sin(odom_th_) + vy * cos(odom_th_)) * dt;
	double delta_th = vth * dt;

	odom_x_ += delta_x;
	odom_y_ += delta_y;
 	odom_th_ += delta_th;
	odom_vx_ = vx;
	odom_vy_ = vy;
	odom_vth_ = vth;

	last_odom_calc_time_ =  current_time;
	last_odom_calc_dist_ = cs.getDistRaced();
}

/////////////////////////////////////////////////////////////////////////////
// publishOdomTF
//
// given a CarState struct, convert it to odom messag 
//
// This code is based on:  
//   http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom 
/////////////////////////////////////////////////////////////////////////////
void RosDriver::publishOdomTF(CarState &cs)
{
	ros::Time current_time = ros::Time::now();

	recalculateOdomValues(cs);

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_th_);

 	//first, we'll publish the transform over tf
 	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "/odom";
 	odom_trans.child_frame_id = "/base_link";

	odom_trans.transform.translation.x = odom_x_;
	odom_trans.transform.translation.y = odom_y_;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	p_tf_broadcaster_->sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = odom_x_;
	odom.pose.pose.position.y = odom_y_;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "/base_link";
	odom.twist.twist.linear.x = odom_vx_;
	odom.twist.twist.linear.y = odom_vy_;
	odom.twist.twist.angular.z = odom_vth_;

	//publish the message
   p_odom_publisher_->publish(odom);
}


/////////////////////////////////////////////////////////////////////////////
// wDrive
//
// wDrive is the key update routine called by the client.cpp mainloop - 
// it give s carstate value and expects a carcontrol response.
//
// what RosDriver does is re-publish the carstate messages and returns the
// last CarControl message it received
//
/////////////////////////////////////////////////////////////////////////////
CarControl RosDriver::wDrive(CarState cs)
{
	ros::Time time_now =  ros::Time::now();

	ros::spinOnce();

	//
	// publish car state messages
	sigevo_champ::CarState msg;
	carStateToMsg(cs, msg);
	msg.stage = stage;
	p_car_state_publisher_->publish(msg);

	//
	// publish laser scan messages
	sensor_msgs::LaserScan scan;
	ros::Duration time_increment = time_now - last_car_state_msg_time_;
	carStateToLaserScan(cs, time_now, time_increment.toSec(), track_sensor_angle_min_, track_sensor_angle_max_, &scan);
	p_laser_publisher_->publish(scan);

	// 
	// occasionally publish laser transforms	
	if (ros::Time::now() >= time_for_next_laser_tf_broadcast_)
	{
		publishLaserTF();
		time_for_next_laser_tf_broadcast_ = ros::Time::now() + ros::Duration(1.0/BROADCAST_LASER_TF_FREQ); 
	}

	// 
	// occasionally publish odometry transforms	
	if (ros::Time::now() >= time_for_next_odom_tf_broadcast_)
	{
		publishOdomTF(cs);
		time_for_next_odom_tf_broadcast_ = ros::Time::now() + ros::Duration(1.0/BROADCAST_ODOM_TF_FREQ); 
	}

	// let ros do it's work
	ros::spinOnce();

	// track the last update - used to determine the time since the last scan data 
	last_car_state_msg_time_ = time_now;

	// return the last car_control message received (see carControlCallback above)
	return cur_car_control_;
}
	
// Callback function called at shutdown
void RosDriver::onShutdown()
{
	// nothing specific needs to be done
}
	
// Callback function called at server restart
void RosDriver::onRestart()
{
	// nothing specific needs to be done
}
