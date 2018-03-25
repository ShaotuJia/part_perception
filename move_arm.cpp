

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


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

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();



  /* The id of the object is used to identify it. */
  collision_object.id = "box1";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;


  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.6;
  box_pose.position.y = -0.4;
  box_pose.position.z =  1.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;


  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

}


