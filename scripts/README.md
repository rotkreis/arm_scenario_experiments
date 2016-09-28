# Description

This folder contains executable source files (ie scripts, here in python). Usually, one script corresponds to one ROS node. They can be run using `rosrun arm_scenario_experiments <script_name>`

# Content 

**This script folder is organized as follow:**

The scripts living directly in this folder are generic (not experiment specific).

Scripts belonging to a particular experiment are stored in a dedicated sub-folder (but are run with rosrun as if they were directly here).

**More details about the experiment-specific scripts in the README of the experiment folder.**

# Details about generic scripts :

* `recorder`
Allows to record topics while giving the ability to another node to control (when starting/when stopping/in which file) to record via services.
Take as argument a rate, a path to a directory where to save bag files in and a list of topics.
  + By default, the recorder does not write to a bagfile the messages published on the topics. A call to the services `recorder/new_bag` and `recorder/start_recording` have to be done (by another node or through a `rosservice call /recorder/new_bag <name>` in a terminal )
  + If an additional argument --start is given, then the recorder will start immediatly recording in a bag file named 'record'.
  
* `inspect_bag`
Simply display the timestamp of every message in the bag.

* `rebag`
This fix script is made to reformat bag files that have been recording raw image topics instead of compressed image topics.

**More details in the doscstrings and comments of each script**
