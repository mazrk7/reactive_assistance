# Reactive Obstacle Avoidance

Repository contains a ROS package that is an adapted implementation of a "gap-based" algorithm for reactive obstacle avoidance, originally introduced as [admissible gap navigation](https://www.sciencedirect.com/science/article/pii/S0921889017306905#!). 

The package currently offers two use-cases for differential-drive rectangular mobile bases with full field-of-view 2D planar laser scan data:
- **Shared Control:** Adjust the input command velocties of a mobile base into safe output commands that avoid colliding with obstacles and navigate towards "gaps" in the environment
- **Autonomous:** Publish goal poses on the appropriate topic and the mobile base will autonomously navigate towards this goal

## High-level Information

In the `docs` folder of this repository, there is a presentation describing this method and other local motion planning techniques for planar mobile robots.

The shared control method has also been applied for safe navigation in a research project involving a robotic wheelchair, which is presented in the following paper:

```
@inproceedings{Zolotas2019,
    author = {Zolotas, M and Demiris, Y},
    booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    pages = {3020--3026},
    title = {{Towards Explainable Shared Control using Augmented Reality}},
    year = {2019}
}
```

## Implementation Details

See an example of how to configure this package for use in the `example.launch` file.

Some additional constraints/considerations before using this package:
- No tests have been made in the gap detection routines for laser scans _without_ a full field-of-view
- It's assumed that the laser data has been filtered already to _not_ contain range values within the robot's footprint

This is a project regularly undergoing development and any contributions/feedback will be well-received. There is also a presentation in the `docs` directory for higher-level understanding of how this package operates.

### TurtleBot3 Configuration

You can also try out the TurtleBot3 configuration example by running the `turtlebot3_example.launch`. I followed [this blog](https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/) to conduct the tests in simulation.

First, launch the robot in Gazebo:
```shell
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

And then run the obstacle avoidance method coupled with the teleoperation and robot description launches:
```shell
roslaunch reactive_assistance turtlebot3_example.launch
```
