#include <ros/ros.h>

// Include maps with robot poses
#include "toolchanger_poses.hpp"

// Include custom support class for moving robot
#include "toolchanger_demo.hpp"

int main(int argc, char* argv[])
{
  // ros init
  ros::init(argc, argv, "igloo_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Load custom class that handles movement
  ToolChangerDemo::Movement movement;
  // Load custom class that handles service calls to toolchanger
  ToolChangerDemo::Tool tool(nh);

  // Don't plan or execute without user Ok
  movement.promptPlan = true;
  movement.promptExecute = true;

  // Determine if we are picking up or setting down a tool
  bool hasTool = tool.status();

  // Bundle staging poses to simplify code
  std::map<std::string, double> staging[2];
  staging[0] = ToolPoses::staging_NoTool;
  staging[1] = ToolPoses::staging_Tool;

  // Goto stageing pose
  movement.planAndExecuteMotion(staging[hasTool], 0.02);

  // Goto lifted tool pose
  movement.planAndExecuteMotion(ToolPoses::hover, 0.01);

  // Goto tool parked pose
  movement.planAndExecuteMotion(ToolPoses::parked, 0.01);

  // toggle toolchanger
  if (tool.toggle(hasTool))
  {
    hasTool = !hasTool;
  }
  else
  {
    ROS_ERROR("Failed to toggle tool");
    return -1;
  }

  // Goto lifted tool pose
  movement.planAndExecuteMotion(ToolPoses::hover, 0.01);

  // Goto staging pose
  movement.planAndExecuteMotion(staging[hasTool], 0.02);

  // Goto home pose
  movement.planAndExecuteMotion("home", 0.05);

  // Try to toggle toolchanger. Should fail as we are too far away from toolchanger
  if (tool.toggle(hasTool))
  {
    ROS_ERROR("We succeded where we should have failed");
  }
  else
  {
    ROS_WARN("Unable to trigger toolchanger");
  }
}
