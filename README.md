# UTS assignment 3: Group Project: Where’s Wally the Brick?

For all tasks it it necessary to initialize the gazebo simulation, by doing the following command in a console.

> roslaunch brick_search brick_search.launch

## Task 1: 2D LiDAR SLAM with teleoperation

To run this task we need to execute the following command.
> roslaunch brick_search mapping

then we need to teleop the robot by executing this command.

> rosrun teleop_twist_keyboard teleop_twist_keyboard.py

After mapping the complete environment we need to save the map by doing the following command.

> rosrun map_server map_saver -f <map_path>

## Task 2: Localize and navigate with a pre-built map


To run this task we need to execute the following command.

> roslaunch brick_search move_localize.launch

After that, we can place 2D_Nav_Goal in Rviz where the robot can move

## Task 3a: Exploration using a ROS package

This program need the `explore lite` package, to install it we need to execute the following command.

> sudo apt install ros-melodic-explore-lite

To run this task we need to execute the following command.

> roslaunch brick_search exploration.launch

This code will autonomaticly make the robot explore the environment.

## Task 3b: Custom decision making

To run this task we need to execute the following command.

> roslaunch brick_search exploration_custom.launch

This code will autonomaticly make the robot explore the environment. It will also display an image of the frontiers in the map.

## Task 4 & Extensions 1/2: Detect Wally the Brick

To run this task we need to execute the following command.

> roslaunch brick_search detect.launch

Then we need to run the brick_search python file.

> rosrun brick_search brick_search.py

After that the robot will turn around itself to localize and then to detect the brick.
When the brick is seen by the robot it will navigate to it, and then display a marker to rviz.

