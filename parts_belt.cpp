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



/*
 * @breif this constructor function maintain belt_inventory and publish topic /belt_inventory
 * 1. receive message from /tf
 * 2. publish belt_inventory
 * @param node ros nodehandle
 */
Belt_Inventory::Belt_Inventory(ros::NodeHandle node):node(node) {

	belt_inventory_publisher = node.advertise<tf2_msgs::TFMessage>("belt_inventory", 1000);

	// set nodehandle for this program

	// subscribe /tf and call functions via function build_belt_inventory
	// ros::Subscriber tf_sub = node.subscribe("/tf", 1000, &Belt_Inventory::build_belt_inventory, this);

	// ros::spin();

}


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


	// send log message
	// ROS_INFO_STREAM("belt_velo_compute start !!" );

	// get last element from old vector belt_inventory
	auto old_last_element = belt_inventory.back();

	// initial belt velocity
	double belt_velo = 0;

	// time difference between new_timestamp and old_timestamp for same part
	double time_stamp_diff = 0;

	// y - position difference between new_y_poistion and old_y_position
	double position_diff = 0;

	// find same part in coming msg
	for (auto possible_part : msg->transforms) {

		if (possible_part.child_frame_id != old_last_element.child_frame_id) {
			break;
		} else {

				// time = stamp.sec + stamp.nsec/(10^9)
				auto possible_part_timestamp = possible_part.header.stamp.sec \
						+ possible_part.header.stamp.nsec/(pow(10, 9));

				// auto possible_part_timestamp = possible_part.header.stamp.toSec();

				auto old_last_element_timestamp = old_last_element.header.stamp.sec \
						+ old_last_element.header.stamp.nsec/(pow(10, 9));
				// auto old_last_element_timestamp = old_last_element.header.stamp.toSec();

				// set up the time stamp difference
				time_stamp_diff = possible_part_timestamp - old_last_element_timestamp;

#if 0

				ROS_INFO_STREAM("old_last_element_timestamp = " << old_last_element_timestamp);

				ROS_INFO_STREAM("possible_part_timestamp = " << possible_part_timestamp);

				ROS_INFO_STREAM("time difference = " << time_stamp_diff);
#endif


				if (time_stamp_diff == 0) {

					ROS_INFO_STREAM(" No belt_velo update !!");
					break;
				} else {
					position_diff = possible_part.transform.translation.y \
							- old_last_element.transform.translation.y;

					 current_belt_velo = position_diff / time_stamp_diff;	// update current belt_velo
					 current_time_stamp_sec = possible_part.header.stamp.sec;			// update current time stamp
					 current_time_stamp_nsec = possible_part.header.stamp.nsec;

					 // compute average belt velocity
					 accumulate_velo += current_belt_velo;
					 velo_measure_times += 1;
					 average_belt_velo = accumulate_velo/velo_measure_times;

#if 0

					ROS_INFO_STREAM("old_last_element_position: [" << old_last_element.transform.translation.x \
							<< " , " << old_last_element.transform.translation.y << " , " \
							<< old_last_element.transform.translation.z);

					ROS_INFO_STREAM("possible_part_position: [" << possible_part.transform.translation.x \
							<< " , " << possible_part.transform.translation.y << " , " \
							<< possible_part.transform.translation.z);


					ROS_INFO_STREAM("Average_Belt_velo = " <<  average_belt_velo);
#endif

					// update last element in belt_inventory
					belt_inventory.back() = possible_part;
				}

		}
	}

}

// callback function part_detect filters information about parts
// from other transfromation in /tf
void Belt_Inventory:: part_detect(const tf2_msgs::TFMessage::ConstPtr& msg) {


	// send log message
	// ROS_INFO_STREAM("part_detect function invoked !!");

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

#if 0
	// compute belt_velo
	if (!belt_inventory.empty()) {
		belt_velo_compute(msg);
	}
#endif


}


/*
 * @brief This function removes the first part that is far away on the conveyer belt
 * @param distance; the distance between logical_camera_1 and part
 * @return void; but update belt_inventory vector
 */
void Belt_Inventory::remove_part_from_belt(const double& distance) {

	// ROS_INFO_STREAM("remove_part_from_belt invoked !!");

	// get the first element(farest element) from belt_inventory
	auto far_element = belt_inventory.begin();

	auto time = ros::Time::now();

	auto predicate_far_element = predicate_part_position(*far_element, time);

	if (predicate_far_element.transform.translation.y < distance) {

			ROS_INFO_STREAM("Far predicate part position y = " << \
					predicate_far_element.transform.translation.y);

			belt_inventory.erase(belt_inventory.begin());
			ROS_INFO_STREAM("REMOVE FAR Part");
		}

}

/**
 * @brief if part out of range return true; else return false
 */
bool Belt_Inventory::out_of_range(const geometry_msgs::TransformStamped& predicate_part) {

	if (predicate_part.transform.translation.y < farthest_distance) {
		return true;
	} else {
		return false;
	}
}


/**
 * @breif predicate the part position on the belt
 * @param part the info of part
 * @param time the time you want to need the position of part
 * @return predicated position and timestamp
 */

geometry_msgs::TransformStamped Belt_Inventory::predicate_part_position(const geometry_msgs::TransformStamped& part, ros::Time time) {

	// compute the time difference
	double duration = time.toSec() - part.header.stamp.toSec();

	// predicate_part
	geometry_msgs::TransformStamped predicate_part;

 	predicate_part.transform.translation.y = part.transform.translation.y - (fabs(average_belt_velo) * duration);

 	return predicate_part;
}






/**
 * @breif update part position in belt_inventory based on current belt_velo
 * @param  msg the new message from rostopic /tf
 * @detail
 * 1. compute y - position in current time
 * 2. update y - position and timestamp in belt_inventory vector
 * 3. remove part out of range (y - position exceed far_distance)
 *
 */
void Belt_Inventory::update_belt_inventory(std::vector<geometry_msgs::TransformStamped>& inventory_belt) {


	// send log message
	// ROS_INFO_STREAM("update_belt_inventory function invoked !!");

	auto Time = ros::Time::now();	// current time

	// assign belt_inventory to predicate_belt_inventory
	current_belt_inventory = convert_belt_inventory_type_to_publish(inventory_belt);

	// geometry_msgs::TransformStamped temp_part_info;

	 for (auto& part : current_belt_inventory.transforms) {

		 // compute the time difference
		 double duration = Time.toSec() - part.header.stamp.toSec();

		 // send messge
		 ROS_INFO_STREAM("duration = " << duration);

		 // update time
		 // part.header.stamp = Time;
		 part.header.stamp = Time;


		 // update part position
		 part.transform.translation.y = part.transform.translation.y - (fabs(average_belt_velo) * duration);

		 // push temp_part
	 }


}


/**
 * @brief convert data type from std::vector to tf2_msgs
 */
tf2_msgs::TFMessage Belt_Inventory::convert_belt_inventory_type_to_publish(const std::vector<geometry_msgs::TransformStamped>& inventory) {

	tf2_msgs::TFMessage publish_data;

	publish_data.transforms = inventory;


	return publish_data;

}




/**
 * @brief publish belt_inventory vector topic belt_inventory
 * 1. set up publisher
 */
void Belt_Inventory::publish_belt_inventory(const int& freq) {

	// send log message
	// ROS_INFO_STREAM(" publish_belt_inventory function Invoke !!");

	// convert belt_inventory to output type tf2_message
	// tf2_msgs::TFMessage publish_data = convert_belt_inventory_type_to_publish(belt_inventory);

	// output size of belt_inventory vector
	ROS_INFO_STREAM("Publish belt_inventory size = " << belt_inventory.size());

	// output size of publish data
	ROS_INFO_STREAM("Publish Size of publish data = " << current_belt_inventory.transforms.size());


	if (!current_belt_inventory.transforms.empty()) {
			 belt_inventory_publisher.publish(current_belt_inventory);
	}


}




/**
 * @brief combine all functions in this class to build belt inventory
 * @param msg the new from rostopic /tf
 * 1. receive msg from /tf topic using callback function
 * 2. detect part from belt using function 'part_detect'
 * 3. compute belt_velo using belt_velo_compute if belt_inventory is not empty
 * 4. remove unpickable part from belt_inventory list using 'remove_part_from_belt'
 * 5. update the position of part in belt_inventory using 'update_belt_inventory'
 * 6. publish a topic 'belt_inventory' to ros master
 *
 */
void Belt_Inventory::build_belt_inventory(const tf2_msgs::TFMessage::ConstPtr& msg) {


	auto recieved_msg = msg;

	// detect part from belt using function 'part_detect'
	part_detect(recieved_msg);

	// if belt_inventory is not empty
	if (!belt_inventory.empty()) {

		// compute belt_velo
		belt_velo_compute(recieved_msg);

		// update the position of part in belt_inventory using 'update_belt_inventory'
		update_belt_inventory(belt_inventory);

		// remove unpickable part from belt_inventory list using 'remove_part_from_belt'
		remove_part_from_belt(farthest_distance);

		 // publish belt_inventory vector
		publish_belt_inventory(publish_freq);

	}

}






