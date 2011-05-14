/***************************************************************************
 
    file                 : RosDriver.h
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
#ifndef ROSDRIVER_H_
#define ROSRIVER_H_

#include "WrapperBaseDriver.h"

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sigevo_champ/CarControl.h"
#include <vector>

////////////////////////////////////////////////////////////////////////////
// class RosDriver
//
// Responsibilities:
//    The RosDriver inherits from the SIG EVO championship Driver classes. 
//    It allows the race car drivers from the SIG EVO/torcs system to be
//    integrated with the ROS system. 
//    
//    Normally a SIG EVO Driver classes would implement driving behaviours by 
//    returning CarControl messages in response to CarState messages.
//
//    However, the RosDriver republishes the CarState messages, and also 
//    publishes LaserScan, Transform and Odometry messages so that it
//    the race car can be treated as a normal robot, using the ROS tools.
//
//    The RosDriver subscribes to CarControl messages and then forwards, the
//    latest CarControl message to the driver server.
//            
// Collaborators: 
//   The main loop in client.cpp is responsible for initialzing this class and also 
//   invoking the wDrive method 
//
// Other references:
//   For an implementation of a driver that uses the messages received and
//   published by RosDriver - have a look at the simple driver code.
//
////////////////////////////////////////////////////////////////////////////

class RosDriver : public WrapperBaseDriver
{
public:
	RosDriver();
	
	// Default Destructor;
	virtual ~RosDriver();

	// Rosdriver needs to initialize from the commandline
	void init0(int argc, char **argv);
	
	virtual void init(float *angles);
	
        // drive function that exploits the CarState and CarControl wrappers as input and output.
        virtual CarControl wDrive(CarState cs);

	// Callback function called at shutdown
	virtual void onShutdown();
	
	// Callback function called at server restart
	virtual void onRestart();
private:

	void publishLaserTF();
	void publishOdomTF(CarState &cs);
	void recalculateOdomValues(CarState &cs);
	void carControlCallback(const sigevo_champ::CarControl::ConstPtr &msg);

	// ros handles to setup the publishing and subscribing
	ros::NodeHandle *p_nodehandle_;
	ros::Publisher  *p_car_state_publisher_;
	ros::Publisher  *p_laser_publisher_;
	ros::Publisher  *p_odom_publisher_;
	ros::Subscriber *p_car_control_subscriber_;

	tf::TransformBroadcaster *p_tf_broadcaster_;
	ros::Time time_for_next_laser_tf_broadcast_;
	ros::Time time_for_next_odom_tf_broadcast_;

	// odometry information is inferred from car state
	// these variables track this
	bool first_odom_calced_;
	ros::Time last_odom_calc_time_;
	double last_odom_calc_dist_;
 	double odom_x_;
        double odom_y_;
        double odom_th_;
 	double odom_vx_;
        double odom_vy_;
        double odom_vth_;

	// when the last sensor data was sent - used to calculate deltas 
	ros::Time last_car_state_msg_time_;

	// track sensor configuration values
	// default is -pi/2,pi/2
	double track_sensor_angle_min_;
	double track_sensor_angle_max_;

	// the last car control message received. it's the one that
	// will be send the next time wDrive is received
	CarControl cur_car_control_;	
};
#endif /*ROSDRIVER_H_*/
