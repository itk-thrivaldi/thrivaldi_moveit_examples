#include <ros/ros.h>

// Used for robot movement
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Used for tool service calls
#include <kuka_kvp_hw_interface/lockTool.h>
#include <kuka_kvp_hw_interface/unlockTool.h>
#include <kuka_kvp_hw_interface/getToolStatus.h>
namespace ToolChangerDemo
{
class Tool
{
private:
  ros::NodeHandle nh_;

  kuka_kvp_hw_interface::lockTool lockMsg;
  kuka_kvp_hw_interface::unlockTool unlockMsg;
  kuka_kvp_hw_interface::getToolStatus statusMsg;

  ros::ServiceClient tool_status;
  ros::ServiceClient tool_lock;
  ros::ServiceClient tool_unlock;

public:
  Tool(ros::NodeHandle nh) : nh_(nh)
  {
    // Create service client
    tool_status = nh_.serviceClient<kuka_kvp_hw_interface::getToolStatus>("gantry/get_tool_status");
    tool_lock = nh_.serviceClient<kuka_kvp_hw_interface::lockTool>("gantry/lock_tool");
    tool_unlock = nh_.serviceClient<kuka_kvp_hw_interface::unlockTool>("gantry/unlock_tool");
  }

  bool close()
  {
    if (tool_lock.call(lockMsg))
    {
      return lockMsg.response.code.val == industrial_msgs::ServiceReturnCode::SUCCESS;
    }
    else
    {
      ROS_ERROR("Failed to call toolchanger service. Aborting");
      exit(-1);
    }
  }
  bool open()
  {
    if (tool_unlock.call(unlockMsg))
    {
      return lockMsg.response.code.val == industrial_msgs::ServiceReturnCode::SUCCESS;
    }
    else
    {
      ROS_ERROR("Failed to call toolchanger service. Aborting");
      exit(-1);
    }
  }
  bool toggle(bool lock)
  {
    if (lock)
    {
      return close();
    }
    else
    {
      return open();
    }
  }

  bool status()
  {
    if (tool_status.call(statusMsg))
    {
      switch (statusMsg.response.status.val)
      {
        case industrial_msgs::TriState::CLOSED:
          return true;
          break;
        case industrial_msgs::TriState::OPEN:
          return false;
          break;
        case industrial_msgs::TriState::UNKNOWN:
          ROS_ERROR("Unable to get status of toolchanger. Aborting");
          exit(-1);
        default:
          ROS_ERROR("Unknown TriState. Aborting");
          exit(-1);
      };
    }
    else
    {
      ROS_ERROR("Failed to reach tool service. Aborting");
      exit(-1);
    }
  }

};  // class

class Movement
{
private:
  moveit_visual_tools::MoveItVisualTools visual_tools;
  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr robot_model;
  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::core::JointModelGroup* joint_model_group;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

public:
  bool promptPlan = false;
  bool promptExecute = false;

  Movement() : visual_tools("root"), robot_model_loader("robot_description"), move_group("gantry_with_manipulator")
  {
    // Delete any old markers
    visual_tools.deleteAllMarkers();
    // Load remote control
    visual_tools.loadRemoteControl();

    visual_tools.trigger();

    robot_model = robot_model_loader.getModel();
    joint_model_group = robot_model->getJointModelGroup("gantry_with_manipulator");
  }

  bool planAndExecuteMotion(std::map<std::string, double> joint_values, double scalingFactor = 1)
  {
    // Limit joint speed
    move_group.setMaxVelocityScalingFactor(scalingFactor);
    move_group.setMaxAccelerationScalingFactor(scalingFactor);

    // Display prompt and wait for user input
    if (promptPlan)
      visual_tools.prompt("Plan move");

    move_group.setJointValueTarget(joint_values);
    move_group.plan(my_plan);

    // Show trajectory line

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // Wait for user input before proceding
    if (promptExecute)
      visual_tools.prompt("Execute move");

    // Excute the planed move
    return move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  }

  bool planAndExecuteMotion(std::string target, double scalingFactor = 1)
  {
    // Limit joint speed
    move_group.setMaxVelocityScalingFactor(scalingFactor);
    move_group.setMaxAccelerationScalingFactor(scalingFactor);

    // Display prompt and wait for user input
    if (promptPlan)
      visual_tools.prompt("Plan move");

    move_group.setNamedTarget(target);
    move_group.plan(my_plan);

    // Show trajectory line

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // Wait for user input before proceding
    if (promptExecute)
      visual_tools.prompt("Execute move");

    // Excute the planed move
    return move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  }

};  // class
}  // namespace
