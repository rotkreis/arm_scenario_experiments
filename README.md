# Install 

Clone this repository into your catkin workspace "src" folder  
In the catkin_ws folder, run catkin_make.
In arm_scenario_experiments, run ./after_install.sh

# Description

This package contains the code needed to carry out data collection and experiments with the `arm_scenario_simulator` package.
Basically, this fork is only used to generate baxter pushing button data.

# How to augment data a little bit

You can modify color of table in : catkin_ws/src/arm_scenario_simulator/models/DREAM_table/model.sdf
just change line 36 : 

<name>Gazebo/SkyBlue</name>
to 
<name>Gazebo/Red</name> for exemple

The color of button is selected randomly

If you want to change button position see : 
catkin_ws/src/arm_scenario_experiments/scripts/button_pressing/button_babbler line 46

# Instruction to record sequences :

#Error you can get (because sometimes, ros hates you)

OSError: [Errno 110] Failed to get robot state on robot/state  
Just relauch the script a few times, it will work

Service IK error :