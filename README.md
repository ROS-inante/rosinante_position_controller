# rosinante_position_controller
Very basic position controller for ROSinante (or other holonomic robot platform).

## Overview 
The position controller exposes two interfaces for setting a target position:
1) The position controller subscribes to */target_pose*. New poses received immediately receive old ones.
2) The controller exposes a ros2 action calles *go_to_pose*.

The Pose given must be of type *PoseStamped*. The *frame_id* in the header will determin the TF reference frame that is used. 
The frame attached to the robot is assumed to be *base_link*.