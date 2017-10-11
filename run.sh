#!/bin/bash

# this file should be placed in the catkin_ws or ros_ws directory to run it to generate data record sequences of Baxter pushing a button
for i in {1..150}
do
  rosrun arm_scenario_experiments button_babbler here
  mv here/record_0 "here_keep/waif_${i}"
done
