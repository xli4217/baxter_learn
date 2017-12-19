# Baxter-Moveit Package for Temporal Logic Experimentation (baxter_tl)

## How To Run

### Simulation

1. Go to ROS workspace `cd ~/ros_ws`

2. Setup environment paths `source baxter_autoIP.sh sim`

3. Launch Gazebo `roslaunch baxter_gazebo baxter_world.launch` (For more info refer to (http://sdk.rethinkrobotics.com/wiki/Simulator_Installation))

4. Enable motors `rosrun baxter_tools enable_robot.py -e`

5. Start joint trajectory controller `rosrun baxter_interface joint_trajectory_action_server.py`

6. Launch Moveit `roslaunch baxter_moveit_config baxter_grippers.launch` (For more info on Baxter and Moveit refer to (http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial))

7. Launch baxter_tl package `roslaunch baxter_tl baxter_tl.launch`

### On The Robot
TODO

## Package Information

### Nodes

1. event_publisher (publishes whether an event has happened)

2. marker_publisher (publishes markers for tabletop, goals, event region and the interactive marker)

3. in_goal_server (provides a service to tell whether gripper is in a goal, if yes which goal)

4. will_collide_server (provides a service to tell if the trajectory resulting from two configurations will collide with table)

### Published Topics

1. /baxter_tl/marker_array (visualization_msgs/MarkerArray)
    * information about markers for tabletop, goals and event region

2. /baxter_tl/int_marker_pose (geometry_msgs/Pose)
    * pose information for the interactive marker

3. /baxter_tl/event (std_msgs/Bool)
    * whether or not the interactive marker is inside the event region

### Services
* /baxter_tl/in_goal
    * request: float64[] j
    * response: uint64[] in_goal

     given a set of joint angles j, returns [x1,x2] where if the resulting gripper position is within goal1.then x1=1 otherwise x1=0. Likewise for x2 

* /baxter_tl/will_collide
    * request: float64[] j1, j2
    * response: bool will_collide

     given two sets of joint angles, check to see if path going from j1 to j2 will collide with tabletop