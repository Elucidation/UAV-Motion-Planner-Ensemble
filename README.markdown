[Simultaneous Planning Localization And Mapping For Unmanned Aerial Vehicles](http://rip11.wikidot.com/simultaneous-planning-localization-and-mapping-for-unmanned)
---
Authors: Bill, Kyel, Sam, Will
The goal of the project is to implement a planning technique into the SLAM framework, using a local planner to direct the path towards the global exploration goal. This work is motivated to improve SLAM performance in uncertain or rapidly changing environments, such as search and rescue missions in a forest using a UAV.

This simulator is written as part of a research assignment for the class [Robotics Intelligence & Planning](http://rip11.wikidot.com)

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