 #!/bin/bash

sequence=0

while true
do
	echo "Running experiment simulator to create sequence $sequence"
	#rosrun arm_scenario_experiments button_babbler "record_$sequence"
	#sleep 2
	sequence=$(($sequence+1))
done
