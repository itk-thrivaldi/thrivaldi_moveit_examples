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

void moveSingleAxis(int axis, double value)
{
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

  // Load planning group just for gantry
  moveit::planning_interface::MoveGroupInterface move_group("gantry_with_manipulator");
  moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("gantry_with_manipulator");

  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  // Wait for user input before proceding
  visual_tools.prompt("Plan move");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // Move single axis
  std::vector<double> joint_values = move_group.getCurrentJointValues();

  joint_values[axis] += value;

  move_group.setJointValueTarget(joint_values);
  move_group.plan(my_plan);

  // Show trajectory line

  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // Wait for user input before proceding
  visual_tools.prompt("Execute move");

  // Excute the planed move
  move_group.execute(my_plan);
}

void trippleMove(int j1, int j2, int j3, double value)
{
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

  // Load planning group just for gantry
  moveit::planning_interface::MoveGroupInterface move_group("gantry_with_manipulator");
  moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("gantry_with_manipulator");

  move_group.setMaxVelocityScalingFactor(0.03);
  move_group.setMaxAccelerationScalingFactor(0.03);

  // Wait for user input before proceding
  visual_tools.prompt("Plan move");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // Move single axis
  std::vector<double> joint_values = move_group.getCurrentJointValues();

  joint_values[j1] += value;
  joint_values[j2] += value;
  joint_values[j3] += value;

  move_group.setJointValueTarget(joint_values);
  move_group.plan(my_plan);

  // Show trajectory line

  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // Wait for user input before proceding
  visual_tools.prompt("Execute move");

  // Excute the planed move
  move_group.execute(my_plan);
}

void quadMove(int j1, int j2, int j3, int j4, double value, double value2)
{
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

  // Load planning group just for gantry
  moveit::planning_interface::MoveGroupInterface move_group("gantry_with_manipulator");
  moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("gantry_with_manipulator");

  move_group.setMaxVelocityScalingFactor(0.02);
  move_group.setMaxAccelerationScalingFactor(0.02);

  // Wait for user input before proceding
  visual_tools.prompt("Plan move");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // Move single axis
  std::vector<double> joint_values = move_group.getCurrentJointValues();

  joint_values[j1] += value;
  joint_values[j2] += value;
  joint_values[j3] += value;

  joint_values[j4] += value2;

  move_group.setJointValueTarget(joint_values);
  move_group.plan(my_plan);

  // Show trajectory line

  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // Wait for user input before proceding
  visual_tools.prompt("Execute move");

  // Excute the planed move
  move_group.execute(my_plan);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "igloo_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  
  // X
  moveSingleAxis(0, 0.08);
  moveSingleAxis(0, -0.08);

  // Y
  moveSingleAxis(1, 0.08);
  moveSingleAxis(1, -0.08);

  // Z
  moveSingleAxis(2, 0.08);
  moveSingleAxis(2, -0.08);

  // Move all gantry axes
  trippleMove(0,1,2, 0.08);
  trippleMove(0,1,2, -0.08); 

  // Move gantry + A1
  quadMove(0, 1, 2, 3, 0.08, 0.3);
  quadMove(0, 1, 2, 3, -0.08, -0.3);
}
