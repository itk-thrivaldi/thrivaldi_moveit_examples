# thrivaldi_moveit_examples
Usage examples for Thrivaldi. This repository was formerly known as `thrivaldi_examples`. To showcase simpler, non-moveit related examples, this was renamed `thrivaldi_moveit_examples` and [thrivaldi_examples](https://github.com/itk-thrivaldi/thrivaldi_examples) now contains no moveit dependencies.

## Installation

1. Follow instructions in [thrivaldi_common](https://github.com/itk-thrivaldi/thrivaldi_common)
2. Install `moveit_visual_tools` if not already installed (`sudo apt-get install ros-kinetic-moveit-visual-tools`)
3. Clone [kuka_kvp_hw_interface](https://github.com/itk-thrivaldi/kuka_kvp_hw_interface) into your `catkin_ws/src`
4. **OPTIONAL** If using `descartes` branch: Clone [ROS-I Descartes](https://github.com/ros-industrial-consortium/descartes) into your `catkin_ws/src`

## Example of running
1. `roslaunch robotlab_moveit_config moveit_planning_execution_kvp.launch sim:=true`
2. `rosrun igloo node`
