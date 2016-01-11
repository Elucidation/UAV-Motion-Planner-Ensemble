[Simultaneous Planning Localization And Mapping For Unmanned Aerial Vehicles](http://rip11.wikidot.com/simultaneous-planning-localization-and-mapping-for-unmanned)
---
Authors: Bill, Kyel, Sam, Will. [Final Class Report](http://cloud.github.com/downloads/Elucidation/UAV-Motion-Planner-Ensemble/GT_RIP_F2011_UAV.pdf)

Published in ICRA: Kyel Ok, Sameer Ansari, William Gallagher, William Sica, Frank Dellaert, Mike Stilman, ["Path Planning with Uncertainty: Voronoi Uncertainty Fields." IEEE International Conference on Robotics and Automation 2013.](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6631230)

This project is a MATLAB Simulator for UAV navigation with vision uncertainty using [SLAM](http://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping).
The goal is to implement this on a raptor quadcopter Spring 2012, with the intent to use in search and rescue operations in forested environments.

[More Images & Videos](http://elucidation.github.com/UAV-Motion-Planner-Ensemble/)

![Final simulation Traveled Path](http://i.imgur.com/ASoql.png)

The build progress is documented on the [class wiki](http://rip11.wikidot.com/simultaneous-planning-localization-and-mapping-for-unmanned).

---
The goal of the project is to implement a planning technique into the SLAM framework, using a local planner to direct the path towards the global exploration goal. This work is motivated to improve SLAM performance in uncertain or rapidly changing environments, such as search and rescue missions in a forest using a UAV.

This simulator is written as part of a research assignment for the GeorgiaTech course [Robotics Intelligence & Planning](http://rip11.wikidot.com) (Fall 2011).

The simulator is written as a script with user defined variables, accessible through `simulator_main.m`


Global Planner
---
A Global planner is implemented using a Voronoi diagram with DFS or A* search through nodes to the global goal.
This is updated each turn based on uncertainty and UAV/robot position

Local Planner
---
A local planner implemented using potential field with gradient descent.
Only visible (within a range of UAV) obstacles are considered.
This is updated each turn based on uncertainty and UAV/robot position
