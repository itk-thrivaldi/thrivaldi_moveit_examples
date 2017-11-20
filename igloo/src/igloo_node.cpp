#include <ros/ros.h>
// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <complex>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "igloo_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Init rviz visual tools, and remove any old markers
  moveit_visual_tools::MoveItVisualTools visual_tools("root");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a
  // high level script via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Batch publishing is used to reduce the number of messages being sent to
  // Rviz for large visualizations
  visual_tools.trigger();

  // Load robot model. It is stored on the parameter server as
  // robot_description.
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  auto robot_model = robot_model_loader.getModel();

  // Create move_group for entire robot
  static const std::string PLANNING_GROUP = "robot_bioik";
  auto joint_model_group_robot =
      robot_model->getJointModelGroup(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface move_group_robot(
      PLANNING_GROUP);

  // Move both manipulators to home position
  move_group_robot.setNamedTarget("home");

  // Create and show plan in rviz
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = move_group_robot.plan(my_plan);
  ROS_INFO("Visualizing plan 1 (named goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.

  visual_tools.publishTrajectoryLine(my_plan.trajectory_,
                                     joint_model_group_robot);
  visual_tools.trigger();

  // Wait for user input before proceding
  visual_tools.prompt("next step");

  // Excute the planed move
  move_group_robot.execute(my_plan);

  // Load planning group just for gantry
  moveit::planning_interface::MoveGroupInterface move_group(
      "gantry_with_manipulator");
  auto joint_model_group =
      robot_model->getJointModelGroup("gantry_with_manipulator");
//  move_group.setPlannerId("LazyPRMstarkConfigDefault");

  // Point tool towards work surface
  std::vector<double> joint_values = move_group.getCurrentJointValues();
  joint_values[7] = -1.5708;
  move_group.setJointValueTarget(joint_values);
  move_group.plan(my_plan);

  // Show trajectory line
  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(my_plan.trajectory_,
                                     joint_model_group_robot);
  visual_tools.trigger();

  // Wait for user input before proceding
  visual_tools.prompt("next step");

  // Excute the planed move
  move_group_robot.execute(my_plan);

  // Increase planning time due to more complex path
  move_group.setPlanningTime(10.0);

  move_group.clearPathConstraints();
  move_group.clearPoseTargets();

  // Set constraints to allow rotation around tool
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "gantry_tool0"; //move_group.getEndEffectorLink();
  ocm.header.frame_id = "root";
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 2*3.1415;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);


  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
  geometry_msgs::Pose target_pose;
  // Add current pose as starting point
  waypoints.push_back(start_pose);

  double height = 1.0;
  double width = 1.0;
  int layers = 5;

  // Create spiral
  for (auto z = 0; z < layers; ++z)
  {
    for (auto i = 0; i < 360; ++i)
    {
      target_pose.orientation = start_pose.orientation;
      target_pose.position.z = start_pose.position.z +
                               double(z) * height / layers +
                               i * (height / layers) / 360;
      std::complex<double> position =
          std::polar(width - (double(z) / layers) - i * (width / layers) / 360,
                     double(i) * 0.0174533);
      target_pose.position.y = start_pose.position.y + position.real();
      target_pose.position.x = start_pose.position.x + position.imag();
      waypoints.push_back(target_pose);

      ++i; // quick hack
    }
  }

  // Cartesian motions are frequently needed to be slower for actions such as
  // approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm
  // via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end
  // effector point.
  move_group.setMaxVelocityScalingFactor(0.1);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::RobotTrajectory trajectory_msg;
  double fraction = move_group.computeCartesianPath(waypoints,
                                                    0.01, // eef_step
                                                    5,  // jump_threshold
                                                    trajectory_msg,
                                                    test_constraints,
                                                    true);

  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory robot_trajectory(
      move_group.getCurrentState()->getRobotModel(), "gantry_with_manipulator");

  // Second get a RobotTrajectory from trajectory
  robot_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(),
                                         trajectory_msg);

  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // Fourth compute computeTimeStamps
  success = iptp.computeTimeStamps(robot_trajectory);
  ROS_INFO("Computed time stamp %s", success ? "SUCCEDED" : "FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);

  // Finally plan and execute the trajectory
  plan.trajectory_ = trajectory_msg;
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
           fraction * 100.0);
  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN,
                           rviz_visual_tools::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i),
                                    rviz_visual_tools::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  move_group.execute(plan);

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
}
