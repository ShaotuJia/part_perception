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

class Grip_Part {

private:
	ros::NodeHandle node;
	tf2_msgs::TFMessage part_offset;			// the part offset position referring to gripper (tool0)
	tf2_msgs::TFMessage detect_part;				// the detected part under logical camera 1
	double check_part_upper_bound = 0.55;					// the upper bound of checking part under logical camera 1
	double check_part_lower_bound = 0;					// the lower bound of checking part under logical camera 1

	ros::Publisher part_offset_publisher;			// publisher to publish part offset based on gripper

	std::string gripper_frame = "vacuum_gripper_link";

	tf::TransformListener listener;					// transform listener
public:

	explicit Grip_Part(ros::NodeHandle node);			// initialize constructor

	// this function detect the part attached on gripper and under logical_camera_1;
	void part_detect(const tf2_msgs::TFMessage::ConstPtr& msg);

	bool is_under_camera_1(const geometry_msgs::TransformStamped part, \
			const double& upper_bound, const double& lower_bound);

	void publish_part_offset();

	void get_part_offset(tf2_msgs::TFMessage detect_part);

	void grip_part_call_back(const tf2_msgs::TFMessage::ConstPtr& msg);

};




#endif /* PART_PERCEPTION_INCLUDE_GRIPPER_GRIP_PART_POS_H_ */
