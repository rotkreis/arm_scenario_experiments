#!/bin/bash
for i in {1..150}
do
  rosrun arm_scenario_experiments button_babbler here
  mv here/record_0 "here_keep/waif_${i}"
done
