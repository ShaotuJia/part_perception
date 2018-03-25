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
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Header.h"

// set up global variable belt_inventory
std::vector<geometry_msgs::TransformStamped> belt_inventory;

// callback function part_detect filters information about parts
// from other transfromation in /tf
void part_detect(const tf2_msgs::TFMessage::ConstPtr& msg) {

	for (auto possible_part : msg->transforms) {

		bool on_inventory = false;		// the part whether exist on belt_inventory

		//
		if (possible_part.header.frame_id == "logical_camera_1_frame") {
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

}


// callback function to compute belt velocity
// compute the difference of msg about same part on the belt
void belt_velo(const tf2_msgs::TFMessage::ConstPtr& msg) {

}

int main(int argc, char **argv) {

	// initialize node parts_belt
	ros::init(argc, argv, "parts_belt");

	// set nodehandle for this program
	ros::NodeHandle n;

	// subscribe topic /tf
	ros::Subscriber sub = n.subscribe("/tf", 1000, part_detect);

	ros::spin();
}



