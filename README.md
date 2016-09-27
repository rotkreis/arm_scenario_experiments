# Description

This package contains the code needed to carry out data collection and experiments with the `arm_scenario_simulator` package.

#Content

**The package contains, as many standard ROS package, the following folders :**

- scripts/ : contains executable source files (ie scripts, here in python). Usually, one script corresponds to one ROS node. They can be run using `rosrun <package_name> <script_name>`

- src/ : contains source files. This can be C++ source files that will be compiled (by adding a few lines in the CMakeList.txt) to create an executable ROS node (there is no example of this here). It can also be python packages that will be used by scripts for instance.

- srv/ : contains service messages definition. As a ROS node can create services, ROS needs to know what kind of message this service expects. One can use standard messages types provided with ROS, or one can define its own service messages in this 'srv' folder.


**The top-level files of the packages are :**

- CMakeList.txt : contains the instructions to build the package. It will be automatically used by CMake when a call to `catkin_make` will be made in the catkin_ws directory. Whenever a new message defintion is added, or when a new node's C++ source is added, this files must be modified accordingly (cf ROS tutorial for that).

- package.xml : contains a descrition of the package and its dependencies

- setup.py : necessary for `catkin_make` to install the python package whose source is in 'src'

