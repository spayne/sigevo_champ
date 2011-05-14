/***************************************************************************
 
    file                 : main.cpp
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
#include "sigevo_champ/CarState.h"
#include "sigevo_champ/CarControl.h"
#include "SimpleDriver.h"
#include "ros/ros.h"

class CarStateFunctor
{
public:
	CarStateFunctor(ros::Publisher *p_publisher)
	{
		p_publisher_ = p_publisher;
	}
	void operator()(const sigevo_champ::CarStateConstPtr& message)
	{
		sigevo_champ::CarControl cc;
		cc = driver_.wDrive(*message);
		p_publisher_->publish(cc);
	}
	ros::Publisher *p_publisher_;
	SimpleDriver driver_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_driver");
	ros::NodeHandle nh;

	ros::Publisher publisher = nh.advertise<sigevo_champ::CarControl> ("torcs_car_control",5);  
	CarStateFunctor csb(&publisher);
	ros::Subscriber subscriber = nh.subscribe<sigevo_champ::CarState> ("torcs_car_state",5, csb);

	ros::spin();

	return 0;
}
