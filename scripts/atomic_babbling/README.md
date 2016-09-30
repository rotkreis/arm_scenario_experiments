# Description

This folder contains the script for moving baxter's left arm one joint at a time (hence atomic).
It is for data collection purposes.

The node `atomic_init_pose` puts the robot in a good initial position (this initial position being used by the babbler as a starting point)

The node `atomic_babbler` contains the controller itself. It takes as arguments the list of joints that are moved. (--all to move all of them)

The node `atomic_launch` is a shortcut for quickly launching the different nodes needed for the babbling and the recording for both the real robot and the simulator.
