 #!/bin/bash

function has_command_finish_correctly {
    if [ "$?" -ne "0" ]
    then
        exit
    else
        return 0
    fi
}

while true
do
	echo "Running again"
	rosrun arm_scenario_experiments button_babbler
	#sleep 2
done

	
