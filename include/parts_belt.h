/**
 * @file parts_belt.h
 * @brief This is the header file of source part_belt.cpp
 * @author Shaotu Jia
 */

#ifndef PERCEPTION_INCLUDE_PARTS_BELT_H_
#define PERCEPTION_INCLUDE_PARTS_BELT_H_

#include <ros/ros.h>
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Header.h"
#include "std_msgs/Time.h"




class Belt_Inventory {
private:
	// set up global variable belt_inventory
	std::vector<geometry_msgs::TransformStamped> belt_inventory;
	double current_belt_velo = 0;	// the velocity of conveyer belt
	double current_time_stamp_sec = 0;	// the current_timestamp corresponding to current belt_velocity
	double current_time_stamp_nsec = 0;
	double far_distance = -2.05;	// the farthest distance that part can be picked up by UR10
	int velo_measure_times = 0;	// times of measuring belt velocity
	double accumulate_velo = 0;		// Accumulate all velocity to compute average velocity
	double average_belt_velo = 0;	// the average belt velo


public:
	explicit Belt_Inventory(ros::NodeHandle& node){};

	void belt_velo_compute(const tf2_msgs::TFMessage::ConstPtr& msg);
	void part_detect(const tf2_msgs::TFMessage::ConstPtr& msg);
	void remove_part_from_belt(const double& distance);
	double time_diff(const geometry_msgs::Transform&, const geometry_msgs::Transform&);
	double position_diff(const geometry_msgs::Transform&, const geometry_msgs::Transform&);
	void update_belt_inventory(std::vector<geometry_msgs::TransformStamped> inventory_belt);
	// double linear_distance();

};



#endif /* PERCEPTION_INCLUDE_PARTS_BELT_H_ */
