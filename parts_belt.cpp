/**
 * @brief This code is to detect all parts on the belt
 * 1. obtain the information about logical_camera_1 from rostopic /tf
 * 2. initialize database 'belt_inventory'as priority queue to store the position of each part on the belt
 * 3. push child_frame information to inventory if it does not exist on the database
 * 4. update the database using the timestamp of newest element and conveyer velocity
 * 5. pop the first element on the database if its out of range
 * @author Shaotu Jia
 */

#include <vector>
#include <ros/ros.h>
#include <math.h>
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Header.h"
#include "include/parts_belt.h"



// callback function to compute belt velocity
// compute the difference of msg about same part on the belt
/**
 * @brief compute the belt velocity
 * @param msg the new message from rostopic /tf
 * 1. get the last element from 'belt_inventory' vector
 * 2. find same part in coming new msg
 * 3. compute belt_velocity : (new_y_position - old_y_position)/(new_timestamp - old_timestamp)
 * @result update  belt_velo variable
 */
void Belt_Inventory::belt_velo_compute(const tf2_msgs::TFMessage::ConstPtr& msg) {

	// ROS_INFO_STREAM("belt_velo_compute start !!" );


	// get last element from old vector belt_inventory
	auto old_last_element = belt_inventory.back();

	// initial belt velocity
	double belt_velo = 0;

	// time difference between new_timestamp and old_timestamp for same part
	auto time_stamp_diff = 0;

	// y - position difference between new_y_poistion and old_y_position
	auto position_diff = 0;

	// find same part in coming msg
	for (auto possible_part : msg->transforms) {

		if (possible_part.child_frame_id != old_last_element.child_frame_id) {
			break;
		} else {

				// time = stamp.sec + stamp.nsec/(10^9)
				auto possible_part_timestamp = possible_part.header.stamp.sec \
						+ possible_part.header.stamp.nsec/(pow(10, 9));

				auto old_last_element_timestamp = old_last_element.header.stamp.sec \
						+ old_last_element.header.stamp.nsec/(pow(10, 9));

				// set up the time stamp difference
				time_stamp_diff = possible_part_timestamp - old_last_element_timestamp;

				ROS_INFO_STREAM("time diffference = " << time_stamp_diff);

				if (time_stamp_diff == 0) {

					ROS_INFO_STREAM(" No belt_velo update !!");
					break;
				} else {
					position_diff = possible_part.transform.translation.y \
							- old_last_element.transform.translation.y;

					belt_velo = position_diff / time_stamp_diff;

					ROS_INFO_STREAM("Belt_velo = " <<  belt_velo);

					// update last element in belt_inventory
					belt_inventory.back() = possible_part;
				}

		}
	}

}

// callback function part_detect filters information about parts
// from other transfromation in /tf
void Belt_Inventory:: part_detect(const tf2_msgs::TFMessage::ConstPtr& msg) {

	for (auto possible_part : msg->transforms) {

		bool on_inventory = false;		// the part whether exist on belt_inventory

		//
		if (possible_part.header.frame_id == "logical_camera_1_frame" \
				&& possible_part.child_frame_id != "logical_camera_1_kit_tray_1_frame" \
				&& possible_part.child_frame_id != "logical_camera_1_agv1_frame") {
			for (auto& part : belt_inventory) {;

				if (possible_part.child_frame_id == part.child_frame_id) {
					on_inventory = true;
					break;
				}
			}

			// if the possible_part does not exist in the belt inventory, add it to belt_inventory
			if (!on_inventory) {
				belt_inventory.push_back(possible_part);

				ROS_INFO_STREAM("Find part under logical camera");
				ROS_INFO_STREAM("The size of inventory is " << belt_inventory.size());
			}
		}
	}

	if (!belt_inventory.empty()) {
		belt_velo_compute(msg);
	}

}

#if 0
/*
 * @brief This function removes the first part that is far away on the conveyer belt
 * @param distance; the distance between logical_camera_1 and part
 * @return void; but update belt_inventory vector
 */
void remove_part_from_belt(const double& distance) {
	if (belt_inventory.begin() > distance) {
		belt_inventory.erase(belt_inventory.begin());
	}

}

#endif







