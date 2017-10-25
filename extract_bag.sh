#!/bin/bash

for i in {1..2}
do
  rosrun arm_scenario_experiments button_bag_to_disk "record_${i}.bag"
done
