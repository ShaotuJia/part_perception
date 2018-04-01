/*
 * main_belt_inventory.cpp
 *
 *  Created on: Mar 28, 2018
 *      Author: shaotu
 */


#include <ros/ros.h>
#include "include/parts_belt.h"
#include "part_perception/Inventory_Predication.h"


int main(int argc, char **argv) {

	// initialize node parts_belt
	ros::init(argc, argv, "parts_belt");

	// node handle
	ros::NodeHandle node;

	// instance of class Belt_Inventory
	Belt_Inventory belt_inventory(node);

	// subscribe topic /tf
	ros::Subscriber sub = node.subscribe("/tf", 1000, &Belt_Inventory::build_belt_inventory, &belt_inventory);

	ros::spin();
}

