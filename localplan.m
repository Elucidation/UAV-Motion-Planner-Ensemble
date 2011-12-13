function [path slope] = localplan(pos, goal, obs, closedist)
[path slope] = graddesc(@(xy)calccost(xy, obs, goal, 'linear2', closedist), pos, goal);