/*
 * parts_belt.h
 *
 *  Created on: Mar 25, 2018
 *      Author: shaotu
 */

#ifndef PERCEPTION_INCLUDE_PARTS_BELT_H_
#define PERCEPTION_INCLUDE_PARTS_BELT_H_

#include <ros/ros.h>
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Header.h"

class Belt_Inventory {
private:
	// set up global variable belt_inventory
	std::vector<geometry_msgs::TransformStamped> belt_inventory;
	double belt_velo = 0;	// the velocity of conveyer belt
public:
	explicit Belt_Inventory(ros::NodeHandle& node);

	void belt_velo_compute(const tf2_msgs::TFMessage::ConstPtr& msg);
	void part_detect(const tf2_msgs::TFMessage::ConstPtr& msg);

};



#endif /* PERCEPTION_INCLUDE_PARTS_BELT_H_ */
