/**
 * @brief source file of incorrect_pos.h
 * @author Shaotu Jia
 */

#include <vector>
#include <ros/ros.h>
#include <math.h>
#include "tf2_msgs/TFMessage.h"
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Header.h"
#include "include/incorrect_pos.h"
#include "osrf_gear/Order.h"


  /// Called when a new Order message is received.
 void Incorrect_Pos_AGV::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders.push_back(*order_msg);
  }

 // callback function part_detect filters information about parts
 // from other transfromation in /tf
 void Incorrect_Pos_AGV:: part_detect(const tf2_msgs::TFMessage::ConstPtr& msg) {

 	// send log message
 	// ROS_INFO_STREAM("part_detect function invoked !!");

 	for (auto possible_part : msg->transforms) {

 		bool on_inventory = false;		// the part whether exist on belt_inventory

 		//
 		if (possible_part.header.frame_id == "logical_camera_2_frame" \
 				&& possible_part.child_frame_id != "logical_camera_2_kit_tray_1_frame" \
 				&& possible_part.child_frame_id != "logical_camera_2_agv1_frame") {


 			for (auto& tray_part : received_orders) {;

 				if (possible_part.child_frame_id == ) {
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

 }

/**
 * @brief Check whether the actual position is within the tolerance of desired position
 * @param actual_pose The actual position detected by logical_camera_2
 * @param desired_pose The desired position from /ariac/orders
 * @return bool; true if it is within the tolerance; else false
 */
 bool Incorrect_Pos_AGV::is_within_tolerance(const geometry_msgs::Transform& actual_pose, const geometry_msgs::Pose& desired_pose) {

 }

 /**
  * @brief check whether the actual translation is within the tolerance of translation
  * @param Vector3 actual_trans The actual translation
  * @param Point desired_trans The desired translation point
  * @return true if within the tolerance
  */
 bool Incorrect_Pos_AGV:: is_within_translation_tolerance(const geometry_msgs::Vector3& actual_trans, const geometry_msgs::Point& desired_trans) {

	 double x_diff ;
 }

 /**
  * @breif check whether the actual orientation is within the tolerance of orientation
  * @param Quaternion actual_orient The actual orientation of part
  * @param Quaternion desired_orient The desired orientation of part in order
  * @return true if within the tolerance
  */
 bool Incorrect_Pos_AGV::is_within_orientation_tolerance(const geometry_msgs::Quaternion& actual_orient, const geometry_msgs::Quaternion& desired_orient) {

	 double Rc, Pc, Yc; // Initialize the three variable for the angle in RPY
	 	 tf2::Quaternion current_orientation;
	 	 // tf2::Matrix3x3(current_orientation).getRPY(Rc, Pc, Yc); // transfer orientation from quaternion to RPY
	 	 tf2::Matrix3x3(current_orientation).getRPY(Rc, Pc, Yc);	// transfer orientation from quaternion to RPY

 }


