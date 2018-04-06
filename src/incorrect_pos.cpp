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
#include "AGV_1/incorrect_pos.h"
#include "osrf_gear/Order.h"
#include "part_perception/Incorrect_Part.h"



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

	 // geometry_msgs::TransformStamped pos_to_agv_1;

 	for (auto possible_part : msg->transforms) {

 		//
 		if (possible_part.header.frame_id == "logical_camera_2_frame" \
 				&& possible_part.child_frame_id != "logical_camera_2_kit_tray_1_frame" \
 				&& possible_part.child_frame_id != "logical_camera_2_agv1_frame") {


 			// if cannot find same part type and position in order, push to incorrect_part_pos_agv_1 vector
 			if (!is_part_in_order(possible_part, received_orders)) {

 				// auto temp_part = convert_pos(possible_part, agv_1_reference_frame);

				tf::StampedTransform temp_transform;

				geometry_msgs::TransformStamped temp_part;

				listener.lookupTransform(agv_1_reference_frame, possible_part.child_frame_id, ros::Time(0), temp_transform);

				tf::transformStampedTFToMsg(temp_transform, temp_part);


 				incorrect_part_pos_agv_1.transforms.push_back(temp_part);


 			}

 		}

 	}


 }


 /**
  * @breif Check if there is same part with same position in recieved_order
  * @param test_part: the part will be compared to the received order
  * @param currect_received_orders: the competition order that currently received
  * @return true if find same part type and same position in recieved order; else false
  */
 bool Incorrect_Pos_AGV::is_part_in_order(const geometry_msgs::TransformStamped& test_part,\
		 const std::vector<osrf_gear::Order>& current_received_orders) {

	// ROS_INFO_STREAM("is_part_in_order invoke");

	 // a temporary variable to store part information
	 geometry_msgs::TransformStamped temp_part_pos;

		// check if the possible_part in correct position based on order
		for (auto& order : current_received_orders) {

			for (auto& kit : order.kits) {

				for (auto& part : kit.objects) {

					if (is_type(test_part.child_frame_id, part.type)) {


						// following Four lines to obtain the relative position using tf tree
						tf::StampedTransform temp_transform;

		 				geometry_msgs::TransformStamped temp_part;

		 				listener.waitForTransform(agv_1_reference_frame,test_part.child_frame_id,ros::Time(0), ros::Duration(2.0));

		 				listener.lookupTransform(agv_1_reference_frame, test_part.child_frame_id, ros::Time(0), temp_transform);

		 				tf::transformStampedTFToMsg(temp_transform, temp_part);


		 				if (is_within_tolerance(temp_part.transform, part.pose)) {

							return true;	// if find a part is within the tolerance


						}

					}

				}

			}
		}

		return false;
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


	 ROS_INFO_STREAM("x_diff = " << x_diff);
	 ROS_INFO_STREAM("y_diff = " << y_diff);
	 ROS_INFO_STREAM("z_diff = " << z_diff);

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

	 tf::Matrix3x3(actual_orientation).getRPY(Ra, Pa, Ya);	// transfer orientation from quaternion to RPY


	 double Rd, Pd, Yd; // // Initialize the three variable for the angle in RPY in desired_orient
	 tf::Quaternion desired_orientation;
	 tf::quaternionMsgToTF(desired_orient, desired_orientation);


	 tf::Matrix3x3(desired_orientation).getRPY(Rd, Pd, Yd);

	 double R_diff = fabs(Ra - Rd);
	 double P_diff = fabs(Pa - Pd);
	 double Y_diff = fabs(Ya - Yd);

	 // !!! test !!
	 // double Y_diff = 0;

	 ROS_INFO_STREAM("R_diff = " << R_diff);
	 ROS_INFO_STREAM("P_diff = " << P_diff);
	 ROS_INFO_STREAM("Y_diff = " << Y_diff);



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
  * @param freq: The rate of publishing topic
  */
 void Incorrect_Pos_AGV::publish_incorrect_part(const int& freq) {

	 // send message to debug
	// ROS_INFO_STREAM("publishing incorrect part function invoked !!");

	 if (!incorrect_part_pos_agv_1.transforms.empty()) {

		// ROS_INFO_STREAM("size of incorrect_part_vector = " << incorrect_part_pos_agv_1.transforms.size());

		 agv_1_incorrect_part_publisher.publish(incorrect_part_pos_agv_1);

		 // send message to debug
		 // ROS_INFO_STREAM("publishing incorrect part");
	 }

 }


/**
 * @brief bool version of covert_pos_to_agv_1 function
 */
 bool Incorrect_Pos_AGV::convert_pos_to_agv_1(\
		 const geometry_msgs::TransformStamped part_pos_logical_2,\
		 std::string reference_frame) {

	 tf::StampedTransform transform_part_agv;		// pos to agv_1

	 // geometry_msgs::TransformStamped part_pos_agv_1;

	 // std::string error_msg = nullptr;

	 ROS_INFO_STREAM("Convert_pos_to_agv_1 invoked !");

	 // wait for tf transform
	 listener.waitForTransform(reference_frame,part_pos_logical_2.child_frame_id,ros::Time(0), ros::Duration(10.0));

	 // check if can transform between reference frame to desired frame
	 bool canTrans = listener.canTransform (reference_frame, part_pos_logical_2.child_frame_id, ros::Time::now());

	 if (canTrans) {
		 listener.lookupTransform(reference_frame,part_pos_logical_2.child_frame_id, ros::Time(0), transform_part_agv);

		 tf::transformStampedTFToMsg(transform_part_agv, part_pos_agv_1);

		 ROS_INFO_STREAM("Success Transform !");

		 return true;
	 } else {

		 ROS_INFO_STREAM("Cannot transform !");
		 return false;
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

 /**
  * @brief function of service server to find incorrect part
  * @param req contains bool find_incorrect_part; whether invoke service
  * @param res contains bool success to check if has incorrect part on AGV_1
  * 	string message to give feedback message; part_info give the incorrect part information
  */
bool Incorrect_Pos_AGV::check_parts_pos(part_perception::Incorrect_Part::Request& req, \
			part_perception::Incorrect_Part::Response& res) {



	if (req.find_incorrect_part) {
		if (server_data.transforms.empty()) {
			res.message = "All Parts are in Correct Position on AGV_1";
			res.success = true;
		} else {
			res.message = " Find Incorrect Part on AGV 1";
			res.parts_info = server_data;
			res.success = false;
		}

		return true;
	} else {
		return false;
	}


	return true;
}

/**
 * @brief Obtain data from rostopic /ariac/incorrect_parts_agv_1
 * @param msg received message from topic /ariac/incorrect_part_agv_1
 */
void Incorrect_Pos_AGV::server_data_call_back(const tf2_msgs::TFMessage::ConstPtr& msg) {

	server_data = *msg;

}
