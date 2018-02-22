

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_arm");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");
  group.setGoalTolerance(0.04);

  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  // The reference frame of target is /world
  Eigen::Affine3d pose = Eigen::Translation3d(-0.614, -1.535, 1.472)
                           * Eigen::Quaterniond(-0.026, 0.925, -0.331, -0.183);

  // show reference frame
  ROS_INFO_STREAM("!! Reference frame: " << group.getPlanningFrame());
  ROS_INFO_STREAM("!! End effector link: " << group.getEndEffectorLink());

  group.setPoseTarget(pose);

  group.move();

}


