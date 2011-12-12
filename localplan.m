function [path slope] = localplan(pos, goal, obs)
[path slope] = graddesc(@(xy)calccost(xy, obs, goal, 'linear2'), pos, goal);


