/*
 * main_belt_inventory.cpp
 *
 *  Created on: Mar 28, 2018
 *      Author: shaotu
 */


#include <ros/ros.h>
#include "include/parts_belt.h"


int main(int argc, char **argv) {

	// initialize node parts_belt
	ros::init(argc, argv, "parts_belt");

	// node handle
	ros::NodeHandle node;

	// instance of class Belt_Inventory
	Belt_Inventory belt_inventory(node);


	// subscribe topic /tf
	// ros::Subscriber sub = node.subscribe("/tf", 1000, &Belt_Inventory::part_detect, &belt_inventory);

	ros::Subscriber sub = node.subscribe("/tf", 1000, &Belt_Inventory::build_belt_inventory, &belt_inventory);
	// subscribe topic /tf and publish belt velocity
	// if (!belt_inventory.empty()) {
	//	ros::Subscriber belt = n.subscribe("/tf", 1000, belt_velo_compute);
	// }

	// belt_inventory.publish_belt_inventory(10);		//
	 ros::spin();
}

