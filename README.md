# Reactive Obstacle Avoidance

Repository contains a ROS package that is an adapted implementation of a "gap-based" algorithm for reactive obstacle avoidance, originally introduced as [admissible gap](https://www.sciencedirect.com/science/article/pii/S0921889017306905#!). The citation for this paper is included below:

```
@article{Mujahed2018,
title = "Admissible gap navigation: A new collision avoidance approach",
journal = "Robotics and Autonomous Systems",
volume = "103",
pages = "93 - 110",
year = "2018",
author = "Muhannad Mujahed and Dirk Fischer and Bärbel Mertsching",
}
```

The package currently offers two use-cases for differential-drive rectangular mobile bases with full field-of-view 2D planar laser scan data:
- **Shared Control:** Adjust the input command velocties of a mobile base into safe output commands that avoid colliding with obstacles and navigate towards "gaps" in the environment
- **Autonomous:** Publish goal poses on the appropriate topic and the mobile base will autonomously navigate towards this goal
See an example of how to configure this package for use in the `example.launch` file.

This is a project regularly undergoing development and any contributions/feedback will be well-received.

There is also a presentation in the `docs` directory for higher-level understanding of how this package operates. 
