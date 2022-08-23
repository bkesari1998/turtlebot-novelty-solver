# coffee-bot

This package contains the ROS stack to integrate the RAPid-Learn algorithm within a turtlebot2 using ROS melodic.

## Launching

The turtlebot's computer will be the master computer for the ROS stack. Running `turtlebot.launch` on the turtlebot's computer will set up the kobuki base, the april-tag detection, and the camera. Running a `plan_*.launch` file on the turtlebot's computer will ready the experiment from the selected point in the selected plan. This includes setting the initial position and orientation of the turtlebot on the map, readying the amcl autonomous navigation, and running every service within the stack. On a remote computer, running `remote.launch` will bring up RVIZ. 

## Running

After running the necessary launch files. Run `manager.py` on the turtlebot's computer using either `rosrun` or a python interpreter. 

## Configuration

### April Tags

April tags are used within the environment to update the high level state representation. The information about the state of the world when a tag is viewed is stored in the `object_tag` ROS parameter. This parameter is defined in `tags.yaml`, which is loaded in every iteration of the experiment.

### Agent State

The high level state representation of the agent at the start of an experiment is held in the `agents` ROS parameter. Depeneding on which iteration of the experiment is being run, a different `*_agent_state.yaml` file is loaded.

### Waypoints

Waypoints are defined by the `waypoints` and `waypoint_list` ROS parameters. For each injected novelty, different waypoints are used. The waypoints are defined in `*_waypoints.yaml`. To use a set of waypoints. The name of the waypoints file needs to be provided to the `plan_*.launch` file through the `waypoints_file` argument.

## Services

- action_executor: Executes the action defined by a line of a pddl plan. The input is a list of strings, the list contains the pddl plan line split on whitespace. The service returns success (bool) and message (string). 

- primitive_move_actions: Executes a primitive move action. The input is a string (the primitive action name). The service returns success (bool) and message (string).

- confirm_state: Updates the high level state representation for the agent. Service does not have an input and returns success (bool) and message (string).

- dock: Attempts to dock the turtlebot autonomously. Has no input and returns success (bool) and message (string). Does not update high level state representation.

- undock: Reverses the turtlebot from the dock. Has no input and returns success (bool) and message (string). Does not update high level state representation.

- move_tb: Moves the turtlebot to a specified waypoint on a map. Waypoint must be defined in `waypoints` ROS parameter. Returns success (bool) and message (string). Does not update high level state representation.
