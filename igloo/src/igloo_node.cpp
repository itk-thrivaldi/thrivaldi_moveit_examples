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
  static const std::string PLANNING_GROUP = "gantry_with_manipulator";
  auto joint_model_group =
      robot_model->getJointModelGroup(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface move_group(
      PLANNING_GROUP);

  // Slow down movements from maximum
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  // Move both manipulators to home position
  move_group.setNamedTarget("home");

  // Create and show plan in rviz
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;
  success = move_group.plan(my_plan);
  ROS_INFO("Visualizing plan 1 (named goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.

  visual_tools.publishTrajectoryLine(my_plan.trajectory_,
                                     joint_model_group);
  visual_tools.trigger();

  // Wait for user input before proceding
  visual_tools.prompt("Execute next step");

  // Excute the planed move
  move_group.execute(my_plan);
  // Wait for user input before proceding
  visual_tools.prompt("Plan next step");


  // Point tool towards work surface
  std::vector<double> joint_values = move_group.getCurrentJointValues();
  joint_values[0] = 1.54 / 2; // Stop at the center of E1 axis
  joint_values[8] = 1.5708; // Move cables out of joint
  joint_values[7] = -1.5708; // Point tool towards the floor
  move_group.setJointValueTarget(joint_values);
  move_group.plan(my_plan);

  // Show trajectory line
  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(my_plan.trajectory_,
                                     joint_model_group);
  visual_tools.trigger();

  // Wait for user input before proceding
  visual_tools.prompt("Execute next step");

  // Excute the planed move
  move_group.execute(my_plan);
  // Wait for user input before proceding
  visual_tools.prompt("Plan next step");


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
  ocm.absolute_z_axis_tolerance = 2*3.1415; // Allow rotation around Z axis of tool
  ocm.weight = 1.0;


  // Limit E1 axis movement
  // [position - tolerance_below, position + tolerance_above]
  moveit_msgs::JointConstraint jcm;
  jcm.joint_name = "gantry_joint_e1";
  jcm.position = 1.54 / 2; // Middle of rail
  jcm.tolerance_above = 0.6;
  jcm.tolerance_below = 0.6;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  test_constraints.joint_constraints.push_back(jcm);

  // Slow down robot movement
  move_group.setMaxVelocityScalingFactor(0.01);
  move_group.setMaxAccelerationScalingFactor(0.01);


  // Create vector for holding all cartesian waypoints
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
  geometry_msgs::Pose target_pose;
  // Add current pose as starting point
  //waypoints.push_back(start_pose);

  double height = 0.5;
  double width = 0.6;
  int layers = 4;

  // Create spiral
  for (auto z = 0; z < layers; ++z)
  {
    double current_layer_width = width - (width * double(z) / layers);
    double next_layer_width =  width - (width * double(z+1) / layers);
    double current_layer_diff = current_layer_width - next_layer_width;

    for (auto i = 0; i < 360; ++i)
    {
      target_pose.orientation = start_pose.orientation;
      target_pose.position.z = start_pose.position.z +
                               double(z) * height / layers +
                               i * (height / layers) / 360;
      std::complex<double> position =
          std::polar(current_layer_width - (current_layer_diff * i / 360), double(i) * 0.0174533);
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
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::RobotTrajectory trajectory_msg;
  double fraction = move_group.computeCartesianPath(waypoints,
                                                    0.01, // eef_step
                                                    3,  // jump_threshold
                                                    trajectory_msg,
                                                    test_constraints,
                                                    true); // Check for collision and constraints

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
  //
  success = iptp.computeTimeStamps(robot_trajectory, // Trajectory to compute time stamps for
                                    0.05, // Max velocity scaling factor
                                    0.05);  // Max acceleration scaling factor
                                    
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
  visual_tools.prompt("Execute next step");

  move_group.execute(plan);
  // Wait for user input before proceding
  visual_tools.prompt("Plan next step");


  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
}
