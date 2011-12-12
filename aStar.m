function [plan, explored, current_path] = aStar(current_vertex, vertices, edges, goal, explored, current_path);
% INPUT
% current_vertex	: The indice of the current vertex the UAV occupies
% vertices		: A 2xN matrix containing the xy coordinates of the vertices
% edges			: A NxN adjacency matrix with a value of 1 for an edge or 0 otherwise.
% goal			: A 2x1 vector containing the xy coordinates of the goal
% explored		: A 1xN vector containing the visited vertices
% current_path		: All nodes currently chosen, used for loop detection
% OUTPUT
% plan			: The index of the next node or nodes to take.  Returns -1 if no solution exists or -2 if the solution has been reached
% explored		: An updated version of the input of the same name reflecting the chosen vertex
% current_path		: All nodes currently chosen, used for loop detection

% Set up variables
[junk,n] = size(vertices);
[junk,m] = size(explored);
find = -1;

% Forward search to reach a higher valued node
plan = forward_search(current_vertex, vertices, edges, goal);

% Backwards search if no higher scoring node exists
if (plan == -1)
	for i=1:n
		if (any(explored == i) ~= 1)
			find = i;
			i = n+1;
		end
	end
	if (find ~= -1)
		% Navigate to the first unexplored node.
		% This may increase the time required, so jump immediately to the first unexplored node
		plan = find;
		% plan = depth_first_search(current_vertex, vertices, edges, find,[]);
	else
		plan = -1;
	end
end

% Update explored and current_path variables based on plan
if (plan >= 0)
	[junk,o] = size(plan);
	for i=1:o
		if (any(explored == plan(i)) == 1)
			explored = [[explored];plan(i);];
		end
	end
	if (find == -1)
		current_path = [[current_path];[plan];];
	else
		current_path = plan;
	end
end
end

function [best_vertex] = forward_search(current_vertex, vertices, edges, goal, current_path)
[junk,n] = size(vertices);
min_distance = sqrt((vertices(1,n) - vertices(1,current_vertex))^2 + (vertices(2,n) - vertices(2,current_vertex))^2);
threshold = 5; % If the node is within a certain distance from the goal, consider the problem solved
if (min_distance < threshold)
	best_vertex = -2;
else
	best_vertex = -1;
	for i=1:n
		if (edges(current_vertex,n) == 1)
			score = sqrt((vertices(1,n) - goal(1,1))^2 + (vertices(2,n) - goal(2,1))^2);
			if (score < min_distance)
				if(any(current_path == n) ~= 1) % Detect a loop in the path
					min_distance = score;
					best_vertex = n;
				end
			end
		end
	end
	if (best_vertex == -1)
		return; % No node approaches the goal, inform the planner backwards search is required
	end
end
end

function [score] = score_node(current_vertex, vertices, edges, goal, explored, next_vertex, scoring_method)
% To do: Score the node using a metric other than distance.
% This function will replace lines of code in forward_search for min_distance = ... and score = ... when implemented
end

% Depth first search
% Function is identical to aStar except as follows
% INPUT
% goal		: The indice of the node to find a path to
% OUTPUT	
% path		: The path from the current node to the goal node
function [path] = depth_first_search(current_vertex, vertices, edges, goal,current_path)
% Omitted currently, for the purposes of this project jumping to the first unexplored node is sufficient for testing.
end