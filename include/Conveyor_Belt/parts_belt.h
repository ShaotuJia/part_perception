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
#include "part_perception/Inventory_Predication.h"
#include "part_perception/TwoInts.h"




class Belt_Inventory {
private:

	std::vector<geometry_msgs::TransformStamped> belt_inventory;	// the original data from belt_inventory
	tf2_msgs::TFMessage belt_inventory_publish_data;	// belt_inventory updated and published
	double current_belt_velo = 0;			// the velocity of conveyer belt
	double current_time_stamp_sec = 0;		// the current_timestamp corresponding to current belt_velocity
	double current_time_stamp_nsec = 0;
	const double farthest_distance = -4.2;	// the farthest distance that part can be picked up by UR10
	int velo_measure_times = 0;				// times of measuring belt velocity
	double accumulate_velo = 0;				// Accumulate all velocity to compute average velocity
	double average_belt_velo = 0;			// the average belt velo
	ros::Publisher belt_inventory_publisher;	// publish belt_inventory as rostopic
	ros::NodeHandle node;						// the ros node handle
	const int publish_freq = 10;				// publish rate 10Hz

	tf2_msgs::TFMessage current_belt_inventory;	// predicated belt inventory

	tf2_msgs::TFMessage predicate_belt_inventory;	// belt_inventory predication in future time

	ros::ServiceServer predicate_inventory;			// the service server to predicate part postiion in belt inventory

	// ros::ServiceServer add_server;			// test service server

	const double on_belt_upper_z_limit = -0.025;			// the upper boundary of checking a part on belt \
												// its z-location referring to logcial_camera_1; 0 m
	const double on_belt_lower_z_limit = -0.035;			// the lower boundary of checking a part on belt
												// its z-location referring to logcial_camera_1; 0.04 m

public:
	explicit Belt_Inventory(ros::NodeHandle node);

	void belt_velo_compute(const tf2_msgs::TFMessage::ConstPtr& msg);
	void part_detect(const tf2_msgs::TFMessage::ConstPtr& msg);
	void remove_part_from_belt(const double& distance);
	double time_diff(const geometry_msgs::Transform&, const geometry_msgs::Transform&);
	double position_diff(const geometry_msgs::Transform&, const geometry_msgs::Transform&);
	void update_belt_inventory(std::vector<geometry_msgs::TransformStamped>& inventory_belt);
	void build_belt_inventory(const tf2_msgs::TFMessage::ConstPtr& msg);
	void publish_belt_inventory(const int& freq);

	tf2_msgs::TFMessage  convert_belt_inventory_type_to_publish(\
			const std::vector<geometry_msgs::TransformStamped>& inventory);


	geometry_msgs::TransformStamped predicate_part_position(const geometry_msgs::TransformStamped& part, ros::Time time);

	bool out_of_range(const geometry_msgs::TransformStamped& predicate_part);

	bool find_parts(part_perception::Inventory_Predication::Request& req, \
			part_perception::Inventory_Predication::Response& res);

	bool is_type(std::string part_name, std::string part_type);

	bool add(part_perception::TwoInts::Request  &req,
			part_perception::TwoInts::Response &res);

	bool is_on_belt(const geometry_msgs::TransformStamped part, const double& upper_bound, const double& lower_bound);
};



#endif /* PERCEPTION_INCLUDE_PARTS_BELT_H_ */
