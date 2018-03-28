/*
 * main_belt_inventory.cpp
 *
 *  Created on: Mar 28, 2018
 *      Author: shaotu
 */


#include <ros/ros.h>
#include "parts_belt.h"


int main(int argc, char **argv) {

	// initialize node parts_belt
	ros::init(argc, argv, "parts_belt");

	// set nodehandle for this program
	ros::NodeHandle n;

	//

	// subscribe topic /tf
	ros::Subscriber sub = n.subscribe("/tf", 1000, part_detect);

	// subscribe topic /tf and publish belt velocity
	// if (!belt_inventory.empty()) {
	//	ros::Subscriber belt = n.subscribe("/tf", 1000, belt_velo_compute);
	// }
	ros::spin();
}

