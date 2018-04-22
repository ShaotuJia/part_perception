/**
 * @brief This node checks the part type and position offset referring to gripper(tool0);
 * this node contains one rosservice to check the offset of part referring to gripper
 * @file grip_part_pos.h
 * @author Shaotu Jia
 * @date April 15, 2018
 */
#ifndef PART_PERCEPTION_INCLUDE_GRIPPER_GRIP_PART_POS_H_
#define PART_PERCEPTION_INCLUDE_GRIPPER_GRIP_PART_POS_H_

#include <ros/ros.h>
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "part_perception/Part_Offset_Gripper.h"

class Grip_Part {

private:
	ros::NodeHandle node;
	tf2_msgs::TFMessage part_offset;			// the part offset position referring to gripper (tool0)
	tf2_msgs::TFMessage detect_part;				// the detected part under logical camera 1
	double check_part_upper_bound = 0.65;					// the upper bound of checking part under logical camera 1
	double check_part_lower_bound = 0;					// the lower bound of checking part under logical camera 1

	ros::Publisher part_offset_publisher;			// publisher to publish part offset based on gripper

	const std::string gripper_frame = "tool0";

	tf::TransformListener listener;					// transform listener for part_offset on gripper

	tf::TransformListener logical_camera_listener;	// listener to check position of logical_camera

	tf2_msgs::TFMessage part_offset_server_data;				// data for /ariac/check_part_offset server

	double logical_camera_incorrect_location_tolerance = 0.01;	// the tolerance for incorrect logical_camera location referring to /world

	const std::string world_frame = "world";							// the name of world frame

public:

	explicit Grip_Part(ros::NodeHandle node);			// initialize constructor

	// this function detect the part attached on gripper and under logical_camera_1;
	void part_detect(const tf2_msgs::TFMessage::ConstPtr& msg);

	bool is_under_camera_1(const geometry_msgs::TransformStamped part, \
			const double& upper_bound, const double& lower_bound);

	void publish_part_offset();

	void get_part_offset(tf2_msgs::TFMessage detect_part);

	void grip_part_call_back(const tf2_msgs::TFMessage::ConstPtr& msg);

	bool check_part_offset(part_perception::Part_Offset_Gripper::Request& req, \
				part_perception::Part_Offset_Gripper::Response& res);

	void server_data_call_back(const tf2_msgs::TFMessage::ConstPtr& msg);

	bool is_logical_camera_1_correct_location(const double& tolerance);

};




#endif /* PART_PERCEPTION_INCLUDE_GRIPPER_GRIP_PART_POS_H_ */
