# rmf_free_fleet
Personal testing prototype of free fleet, using new rmf_fleet_adapter api


  --- Test Code ---

## Mind Mapping
Pouring out my thoughts....
This repo is trying to serve the need to create an example of fleet_manager which utilizes the `rmf_fleet_adapter`.
The idea here is to create a FM server which has:
 - A switchable fleet_control_type, e.g. `full_control`, `traffic_light`, `read_only`
 - Primary ROS2 client interface, using the depreciated `rmf_fleet_msgs` for consistency

Goal: 
 - Able to serve as the middleman FM for the slotcar in `rmf_demos`
 - Create a server which applies the traffic_light concept
 - Create an example prototype code, which will eventually merge with the current `osrf/free_fleet`
 - Client interface:
   - This is still in planning, but th desire is to create:
   - 1. Easy deployable ros1 "free_fleet" client (plan: dockerize the "bridge" and deploy on each remote robot)
   - 2. Ros2 client, which can be tested on gazebo slotcar plugin, or even with the developing `navigation2`
   - 3. Switchable client interface.

Architecture

```
       [rmf_core]
          |||
-----------------------
|[ rmf_fleet_adapter ]|
|  free_fleet_server  |
|                     |
-----------------------   
           |
    rmf_fleet_msgs ( ???? tobe decided)
           |
           | type of client interfaces
      ----------------------------------------------------x-----------
      |                    |                   |                     |
[ros1_client]          [ros2_client]       [slotcar_plugins]   [blindcar_plugin]
[move_base_robots]     [nav2_robots]       [ gazebo_robot ]    [gzebo bot, to validate traffic light]

```

Recommendation:
- this is only serving robot which doesnt have a FM. In other words, if your robot fleet consists of a FM,
it is recommended to create your own adapter layer which uses `rmf_fleet_adapter` api
