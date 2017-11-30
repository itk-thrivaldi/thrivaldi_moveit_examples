#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <boost/timer/timer.hpp>
#include <boost/lexical_cast.hpp>
#include <blocking_reader.h>

#define DEG2RAD 0.017453292519943295769236907684886127134428718885417254560

void waitKeypress(std::string msg = "Press any key to continue") {
  std::cout << msg << std::endl;
  std::cin.ignore();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_latency_test");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("Floor");
  group.setMaxVelocityScalingFactor(0.1);  
  group.setMaxAccelerationScalingFactor(0.1);

  std::map<std::string, double> pos1;
  std::map<std::string, double> pos2;

  
  pos1["floor_joint_a1"] = 64*DEG2RAD; 
  pos1["floor_joint_a2"] = -94*DEG2RAD; 
  pos1["floor_joint_a3"] = 72*DEG2RAD; 
  pos1["floor_joint_a4"] = 138*DEG2RAD; 
  pos1["floor_joint_a5"] = -2.6*DEG2RAD; 
  pos1["floor_joint_a6"] = -138*DEG2RAD; 

  pos2["floor_joint_a1"] = 83*DEG2RAD; 
  pos2["floor_joint_a2"] = -90*DEG2RAD; 
  pos2["floor_joint_a3"] = 91*DEG2RAD; 
  pos2["floor_joint_a4"] = 138*DEG2RAD; 
  pos2["floor_joint_a5"] = -2.6*DEG2RAD; 
  pos2["floor_joint_a6"] = -138*DEG2RAD; 

  for(int i=0; i<5; i++) {

    std::cout << "Ready to move to pos" << boost::lexical_cast<std::string>(i%2) << std::endl;
    
  
    moveit::planning_interface::MoveGroup::Plan plan;
    group.setJointValueTarget((i%2) ? pos1 : pos2);
    bool success = group.plan(plan);
    
    if (!success) {
	std::cout << "Failed to plan path" << std::endl;
	break;
    }
    std::cout << "Visualizing plan (pose goal)" << std::endl;
    waitKeypress("Execute plan " + boost::lexical_cast<std::string>(i));
    
    group.execute(plan);
    std::cout << "Move " << i << " done" << std::endl << std::endl;
    ros::Duration(0.5).sleep();
  }
}
