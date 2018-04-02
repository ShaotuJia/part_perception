/**
 * @brief source file of incorrect_pos.h
 * @author Shaotu Jia
 */

#include <vector>
#include <ros/ros.h>
#include <math.h>
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Header.h"
#include "include/incorrect_pos.h"
#include "osrf_gear/Order.h"



Incorrect_Pos_AGV::Incorrect_Pos_AGV(ros::NodeHandle node): node(node) {
	agv_1_incorrect_part_publisher = node.advertise<tf2_msgs::TFMessage>("/ariac/incorrect_parts_agv_1", 1000);

}



  /// Called when a new Order message is received.
 void Incorrect_Pos_AGV::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {

	// ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders.push_back(*order_msg);
  }


 // callback function part_detect filters information about parts
 // from other transfromation in /tf
 void Incorrect_Pos_AGV:: agv_part_detect(const tf2_msgs::TFMessage::ConstPtr& msg) {

 	// send log message
	// ROS_INFO_STREAM("agv_part_detect function invoked !!");

	 // clear previous data in variable
	 incorrect_part_pos_agv_1.transforms.clear();

	 geometry_msgs::TransformStamped pos_to_agv_1;



 	for (auto possible_part : msg->transforms) {

 		//
 		if (possible_part.header.frame_id == "logical_camera_2_frame" \
 				&& possible_part.child_frame_id != "logical_camera_2_kit_tray_1_frame" \
 				&& possible_part.child_frame_id != "logical_camera_2_agv1_frame") {


 			// check if the possible_part in correct position based on order
 			for (auto& order : received_orders) {

 				for (auto& kit : order.kits) {

 					for (auto& part : kit.objects) {

						if (is_type(possible_part.child_frame_id, part.type)) {

							// !! convert possible part to relative pos to agv_1
							pos_to_agv_1 = convert_pos_to_agv_1(possible_part.transform);

							if (!is_within_tolerance(pos_to_agv_1, part.pose)) {

								incorrect_part_pos_agv_1.transforms.push_back(possible_part);
							}

						}
 					}
 				}
 			}

 		}
 	}

 	// send message
 	// ROS_INFO_STREAM("size of incorrect part vector: " << incorrect_part_pos_agv_1.transforms.size());

 	// publish to topic
 	// publish_incorrect_part(publish_rate);

 }

/**
 * @brief Check whether the actual position is within the tolerance of desired position
 * @param actual_pose The actual position detected by logical_camera_2
 * @param desired_pose The desired position from /ariac/orders
 * @return bool; true if it is within the tolerance; else false
 */
 bool Incorrect_Pos_AGV::is_within_tolerance(const geometry_msgs::Transform& actual_pose, const geometry_msgs::Pose& desired_pose) {

	 if (is_within_translation_tolerance(actual_pose.translation, desired_pose.position) && \
			 is_within_orientation_tolerance(actual_pose.rotation, desired_pose.orientation)) {

		 return true;
	 } else {
		 return false;
	 }

 }

 /**
  * @brief check whether the actual translation is within the tolerance of translation
  * @param Vector3 actual_trans The actual translation
  * @param Point desired_trans The desired translation point
  * @return true if within the tolerance
  */
 bool Incorrect_Pos_AGV:: is_within_translation_tolerance(const geometry_msgs::Vector3& actual_trans, const geometry_msgs::Point& desired_trans) {

	 double x_diff = fabs(actual_trans.x - desired_trans.x);
	 double y_diff = fabs(actual_trans.y - desired_trans.y);
	 double z_diff = fabs(actual_trans.z - desired_trans.z);

	 if (x_diff < translation_tolerance && y_diff < translation_tolerance && z_diff < translation_tolerance) {
		 return true;
	 } else {
		 return false;
	 }
 }

 /**
  * @breif check whether the actual orientation is within the tolerance of orientation
  * @param Quaternion actual_orient The actual orientation of part
  * @param Quaternion desired_orient The desired orientation of part in order
  * @return true if within the tolerance
  */
 bool Incorrect_Pos_AGV::is_within_orientation_tolerance(const geometry_msgs::Quaternion& actual_orient, const geometry_msgs::Quaternion& desired_orient) {

	 double Ra, Pa, Ya; // Initialize the three variable for the angle in RPY in actual_orient
	 tf::Quaternion actual_orientation;

	 tf::quaternionMsgToTF(actual_orient, actual_orientation);

	 // actual_orientation.x = actual_orient.x;
	 // actual_orientation.y = actual_orient.y;
	 // actual_orientation.z = actual_orient.z;
	 // actual_orientation.w = actual_orient.w;

	 tf::Matrix3x3(actual_orientation).getRPY(Ra, Pa, Ya);	// transfer orientation from quaternion to RPY


	 double Rd, Pd, Yd; // // Initialize the three variable for the angle in RPY in desired_orient
	 tf::Quaternion desired_orientation;
	 tf::quaternionMsgToTF(desired_orient, desired_orientation);

	 // desired_orientation.x = desired_orient.x;
	 // desired_orientation.y = desired_orient.y;
	 // desired_orientation.z = desired_orient.z;
	 // desired_orientation.w = desired_orient.w;

	 tf::Matrix3x3(desired_orientation).getRPY(Rd, Pd, Yd);

	 double R_diff = fabs(Ra - Rd);
	 double P_diff = fabs(Pa - Pd);
	 double Y_diff = fabs(Ya - Yd);

	 if (R_diff < orientation_tolerance && P_diff < orientation_tolerance && Y_diff < orientation_tolerance) {
		 return true;
	 } else {
		 return false;
	 }


 }

 /**
  * @brief check if the part is desired part type
  * @param part_name The full name of part
  * @param part_type The name of desired type
  * @return true if the part is desired part; else return false
  */
 bool Incorrect_Pos_AGV::is_type(std::string part_name, std::string part_type) {

 	// ROS_INFO_STREAM("part_name: " << part_name);
 	// ROS_INFO_STREAM("part_type: " << part_type);

 	if (part_name.find(part_type) != std::string::npos) {
 		return true;
 	} else {
 		return false;
 	}
 }

 /**
  * @brief Publish incorrect part type and its position
  */
 void Incorrect_Pos_AGV::publish_incorrect_part(const int& freq) {

	 // send message to debug
	// ROS_INFO_STREAM("publishing incorrect part function invoked !!");

	 if (!incorrect_part_pos_agv_1.transforms.empty()) {

		 agv_1_incorrect_part_publisher.publish(incorrect_part_pos_agv_1);

		 // send message to debug
		 ROS_INFO_STREAM("publishing incorrect part");
	 }

 }
 /**
  * @breif convert position from referring logical_camera_2 to agv_1
  * @param part_pos_logical_2; the relative position between part and logical_camera_2
  * @return the part position referring to agv_1
  */

 geometry_msgs::TransformStamped Incorrect_Pos_AGV::convert_pos_to_agv_1(const geometry_msgs::TransformStamped part_pos_logical_2, std::string reference_frame) {

	 tf::StampedTransform transform_part_agv;		// pos to agv_1

	 geometry_msgs::TransformStamped part_pos_agv_1;

	 std::string error_msg = nullptr;

	 // check if can transform between reference frame to desired frame
	 bool canTrans = listener.canTransform (reference_frame, part_pos_logical_2.child_frame_id, ros::Time::now());

	 if (canTrans) {
		 listener.lookupTransform(reference_frame,part_pos_logical_2.child_frame_id, ros::Time(0), transform_part_agv);

		 tf::transformStampedTFToMsg(transform_part_agv, part_pos_agv_1);

		 return part_pos_agv_1;
	 } else {
		 return nullptr;
	 }

 }

 /**
  * @brief the call function to detect and publish incorrect part position
  */

 void Incorrect_Pos_AGV::incorrect_part_call_back(const tf2_msgs::TFMessage::ConstPtr& msg) {

	 // call agv_part_detect function
	 agv_part_detect(msg);

	 // publish incorrect part
	 publish_incorrect_part(publish_rate);


 }
