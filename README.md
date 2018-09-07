# SDPTrajectoryGeneration
This repository is related with the author's master thesis titled "Redundancy control through SDP Lexicographic Optimization".

Based on [Peter Corke Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox) for Matlab, it contains a framework to generate trajectories for a redundant robot. Each task has its own priority and which will be taken into account when solving the trajectory.

Some tasks are already implemented in a 4-link planar manipulator: Pose tracking task (position and orientation), joint limits task as well as an obstacle avoidance task.

A nice exercise to do is to play with the task priorities to check their effect on the trajectory resolution.
