/**
 * @brief This node checks the part type and position offset referring to gripper(tool0);
 * this node contains one rosservice to check the offset of part referring to gripper
 * @file grip_part_pos.cpp
 * @author Shaotu Jia
 * @date April 15, 2018
 */

#include <ros/ros.h>
#include <math.h>
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "Gripper/grip_part_pos.h"

/**
 * @brief this constructor initialize rostopic /ariac/part_offset_gripper
 */
Grip_Part::Grip_Part(ros::NodeHandle node): node(node) {

	part_offset_publisher = node.advertise<tf2_msgs::TFMessage>("/ariac/part_offset_gripper", 1000);
}


/**
 * @brief This function filter detected part from all information of rostopic /tf
 * @param msg; the message recieved from rostoipic /tf
 */
void Grip_Part::part_detect(const tf2_msgs::TFMessage::ConstPtr& msg) {


	// send log message
	// ROS_INFO_STREAM("part_detect function invoked !!");

	for (auto possible_part : msg->transforms) {

		bool on_inventory = false;		// the part whether exist on belt_inventory

		// filter part referring to frame logical_camera_1_frame and also on the convey belt(function is_on_belt())
		if (possible_part.header.frame_id == "logical_camera_1_frame" \
				&& possible_part.child_frame_id != "logical_camera_1_kit_tray_1_frame" \
				&& possible_part.child_frame_id != "logical_camera_1_agv1_frame" \
				&& is_under_camera_1(possible_part, check_part_upper_bound , check_part_lower_bound)) {

			detect_part.transforms.push_back(possible_part);


		} else {
			detect_part.transforms.clear();		// clear vector
			part_offset.transforms.clear();		// clear vector
		}
	}

}

/**
 * @brief check if the part in the correct detection range of logical camera 1
 * for checking the position of part related to gripper (tool0)
 * @param part The detected under logical_camera_1
 * @param upper_bound The upper bound of a part position related to logical_camera_1
 * @param lower_bound The lower bound of a part position related to logical_camera_1
 * @warning: !!!!! when gripper picking part and under logical camera;
 * the height is part.transform.translation.x !!!!
 * @return true if the part is on the belt; else return false
 */
bool Grip_Part::is_under_camera_1(const geometry_msgs::TransformStamped part, \
		const double& upper_bound, const double& lower_bound) {


	// ROS_INFO_STREAM(" part height = " << part.transform.translation.x);

	if (part.transform.translation.x < upper_bound && part.transform.translation.x > lower_bound) {

		// ROS_INFO_STREAM("True \n");
		return true;
	} else {
		// ROS_INFO_STREAM("False \n");
		return false;
	}

}

/**
 * @brief this function publish part offset if there is part under logical camera 1 and on gripper
 */
void Grip_Part::publish_part_offset() {

	if (!part_offset.transforms.empty()) {

		// !! will change to part_off !!
		part_offset_publisher.publish(part_offset);
	}
}

/**
 * @brief Get offset between part and gripper using tf transfer
 * @param detect_part; the part info detected by logical_camera_1
 */
void Grip_Part::get_part_offset(tf2_msgs::TFMessage detect_part) {

	if (!detect_part.transforms.empty()) {

		auto attached_part = detect_part.transforms[0];

		tf::StampedTransform temp_transform;

		geometry_msgs::TransformStamped temp_part;

		listener.waitForTransform(gripper_frame, attached_part.child_frame_id,ros::Time(0), ros::Duration(1));

		listener.lookupTransform(gripper_frame, attached_part.child_frame_id, ros::Time(0), temp_transform);

		tf::transformStampedTFToMsg(temp_transform, temp_part);	// assign value to part_offset

		part_offset.transforms.push_back(temp_part);			// push back tf

	}
}

/**
 * @brief This is the call back function to accomplish functions in this class
 * @param msg; message from rostopic /tf
 */
void Grip_Part::grip_part_call_back(const tf2_msgs::TFMessage::ConstPtr& msg) {

	// detect part under logical_camera_1 and on gripper
	part_detect(msg);

	// get offset part
	get_part_offset(detect_part);

	// publish part offset if gripper is checking parts using logical_camera_1
	publish_part_offset();

}

