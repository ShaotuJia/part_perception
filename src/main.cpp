/*
 * main_belt_inventory.cpp
 *
 *  Created on: Mar 28, 2018
 *      Author: shaotu
 */


#include <ros/ros.h>
#include "Conveyor_Belt/parts_belt.h"
#include "AGV_1/incorrect_pos.h"
#include "Gripper/grip_part_pos.h"
#include "part_perception/Inventory_Predication.h"
#include "part_perception/Incorrect_Part.h"


int main(int argc, char **argv) {

	// initialize node parts_belt
	ros::init(argc, argv, "parts_belt");

	// node handle
	ros::NodeHandle node;

	// instance of class Belt_Inventory
	Belt_Inventory belt_inventory(node);

	// subscribe topic /tf
	ros::Subscriber sub = node.subscribe("/tf", 1000, &Belt_Inventory::build_belt_inventory, &belt_inventory);

	// create ros server
	// ros::ServiceServer server = node.advertiseService("add_two_ints", &Belt_Inventory::add, &belt_inventory);

	ros::ServiceServer inventory_server = node.advertiseService("/ariac/query_belt_part", &Belt_Inventory::find_parts, &belt_inventory);

	// initialize Incorrect_Pos_AGV class
	Incorrect_Pos_AGV incorrect_pos_agv(node);

	// subscribe /ariac/order to received order info
	ros::Subscriber order_sub = node.subscribe("/ariac/orders", \
			1000, &Incorrect_Pos_AGV::order_callback, &incorrect_pos_agv);

	// subscribe /tf to invoke all function to detect all incorrect parts on AGV_1
	ros::Subscriber incorrect_part_sub = node.subscribe("/tf", 1000, &Incorrect_Pos_AGV::incorrect_part_call_back,\
			&incorrect_pos_agv);

	// obtain server data
	ros::Subscriber server_data_sub = node.subscribe("/ariac/incorrect_parts_agv_1", 1000,\
			&Incorrect_Pos_AGV::server_data_call_back, &incorrect_pos_agv);

	// create ros server to check part position on AGV_1
	ros::ServiceServer part_pos_agv_server = node.advertiseService("/ariac/check_part_pos_agv_1", \
			&Incorrect_Pos_AGV::check_parts_pos, &incorrect_pos_agv);

	// initialize Grip_Part class
	Grip_Part grip_part_offset(node);

	// subscribe /tf to check the part offset on gripper
	ros::Subscriber grip_part_pos_sub = node.subscribe("/tf", 1000, &Grip_Part::grip_part_call_back,\
			&grip_part_offset);


	ros::spin();
}

