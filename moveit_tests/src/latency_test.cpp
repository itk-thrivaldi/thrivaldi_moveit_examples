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

	// Setup
	char usb[] = "/dev/ttyACM0";
	int baud = 38400;
	int timeout = 100; // ms
	int num_tests = 10;
	

	// Declare variables
	boost::asio::io_service io;
//	boost::asio::serial_port port(io);
	
	// Connect to serial port	
//	port.open(usb);
//	port.set_option(boost::asio::serial_port_base::baud_rate(baud));
//	blocking_reader reader(port, timeout);

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("Gantry");
  group.setMaxVelocityScalingFactor(0.1);  
  std::map<std::string, double> pos1;
  std::map<std::string, double> pos2;

  
  pos1["gantry_joint_a1"] = -31.95387*DEG2RAD; 
  pos1["gantry_joint_a2"] = -98.78010*DEG2RAD; 
  pos1["gantry_joint_a3"] = 101.0818*DEG2RAD; 
  pos1["gantry_joint_a4"] = 3.209918E-02*DEG2RAD; 
  pos1["gantry_joint_a5"] = -1.140894*DEG2RAD; 
  pos1["gantry_joint_a6"] = -2.575621E-02*DEG2RAD; 
  pos1["gantry_joint_e1"] = 1153.259/1000; 
  pos1["gantry_joint_e2"] = 2800.001/1000; 
  pos1["gantry_joint_e3"] = -749.9890/1000;

  pos2["gantry_joint_a1"] = -31.95387*DEG2RAD; 
  pos2["gantry_joint_a2"] = -103.7478*DEG2RAD; 
  pos2["gantry_joint_a3"] = 104.9128*DEG2RAD; 
  pos2["gantry_joint_a4"] = 3.209918E-02*DEG2RAD; 
  pos2["gantry_joint_a5"] = -1.140894*DEG2RAD; 
  pos2["gantry_joint_a6"] = -2.575621E-02*DEG2RAD; 
  pos2["gantry_joint_e1"] = 1153.259/1000; 
  pos2["gantry_joint_e2"] = 2800.001/1000; 
  pos2["gantry_joint_e3"] = -749.9890/1000;

  char c;
  std::cout << "Test,Time" << std::endl;
  for(int i=0; i<5; i++) {
    // clear serial buffer 
 //   while ( reader.read_char(c) ) {}

    waitKeypress("Ready to move to pos" + boost::lexical_cast<std::string>(i%2));
    
    // Set output and start timer
    std::string format = boost::lexical_cast<std::string>(i) + ",%w\n";
    boost::timer::auto_cpu_timer t(9,format);
  
    // Set target and start nonblocking move
    group.setJointValueTarget((i%2) ? pos1 : pos2);
    group.asyncMove();

    // wait for movement
//    reader.read_char(c);
 
    // Stop timer, write report
    t.stop();
    t.report();
    std::cout.flush();
  }
}
