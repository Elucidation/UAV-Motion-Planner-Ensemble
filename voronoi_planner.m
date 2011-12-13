function [local_goal,termination_flag,VX,VY] = voronoi_planner(trees,robot,goal)
% INPUT
% trees	: An Nx2 vector containing the X-Y coordinates of the trees
% robot	: A 1x2 vector containing the X-Y coordinates of the robot
% goal	: A 1x2 vector containing the X-Y coordinated of the goal
% OUTPUT
% local_goal : The x,y coordinates of the local voronoi vertex
% termination_flag : 1 if the program should return that no path exists, 0
% otherwise
% VX & VY are Voronoi edges for use : 
%           plot(VX,VY,'-');
%           set(h(1:end-1),'xliminclude','off','yliminclude','off')

% Set up local variables, to be configured for specific runs
threshold = 1;
termination_flag = 0;
local_goal = [];

% Draw the vertices of regular polygons around the robot and goal to
% represent them in teh voronoi space
o_trees = trees; % Placeholder to format trees if input does not match expected
num_trees = size(o_trees,1); %[num_trees,junk] = size(o_trees);
objects = [o_trees];
temp = robot;
%objects=[[objects];[temp(1,1)+1,temp(1,2)-1)];[temp(1,1),temp(1,2)+1];[temp(1,1),temp(
objects = [[objects];[temp(1,1)+0.1,temp(1,2)];[temp(1,1)+0.05,temp(1,2)+0.1];[temp(1,1)-0.05,temp(1,2)+0.1];[temp(1,1)-0.1,temp(1,2)];[temp(1,1)-0.05,temp(1,2)-0.1];[temp(1,1)+0.05,temp(1,2)-0.1];];
temp = goal;
objects = [[objects];[temp(1,1)+0.1,temp(1,2)];[temp(1,1)+0.05,temp(1,2)+0.1];[temp(1,1)-0.05,temp(1,2)+0.1];[temp(1,1)-0.1,temp(1,2)];[temp(1,1)-0.05,temp(1,2)-0.1];[temp(1,1)+0.05,temp(1,2)-0.1];];

% Draw bounding box of area
temp = [robot(1,1),goal(1,2)];
objects = [[objects];[temp(1,1)+0.1,temp(1,2)];[temp(1,1)+0.05,temp(1,2)+0.1];[temp(1,1)-0.05,temp(1,2)+0.1];[temp(1,1)-0.1,temp(1,2)];[temp(1,1)-0.05,temp(1,2)-0.1];[temp(1,1)+0.05,temp(1,2)-0.1];];
temp = [goal(1,1),robot(1,2)];
objects = [[objects];[temp(1,1)+0.1,temp(1,2)];[temp(1,1)+0.05,temp(1,2)+0.1];[temp(1,1)-0.05,temp(1,2)+0.1];[temp(1,1)-0.1,temp(1,2)];[temp(1,1)-0.05,temp(1,2)-0.1];[temp(1,1)+0.05,temp(1,2)-0.1];];

% Debug line to show points
[VX, VY] = voronoi(objects(:,1),objects(:,2));

% SAM - Keep only unique along x/y
objects = unique(objects,'rows');

% Perform Voronoi decomposition of the explored region
[v,c] = voronoin(objects);

% Obtain edges from the Voronoi vertices output by Voronoi decomposition
edges = []; 
[n,junk] = size(v);
v = v(2:n,:);
[n,junk] = size(v);
edges = zeros(n,n);
[m,junk] = size(c);
for i=1:m
    [junk,k] = size(c{i});
    for j=1:k
        if (j == k)
            if(c{i}(1,1) ~= 1 && c{i}(1,j) ~= 1)
                edges(c{i}(1,1)-1,c{i}(1,j)-1) = 1;
            end
        elseif (c{i}(1,j) ~=1 && c{i}(1,j+1) ~= 1)
            edges(c{i}(1,j)-1,c{i}(1,j+1)-1) = 1;
        end
    end
end

% Remove edges from the adjacency matrix which are too close to trees
% The runtime of this is abysmal without kd-trees
% Every edge must be checked against every vertex for a maximum runtime of n^3
% We discussed a method which checks for proximity to two points, but the runtime of this is n^4 and this should give similar results
for i=1:n
    for j=1:n
        if(edges(i,j) == 1)
            for k=1:num_trees
                % Check if edge k is in the bounding box of i and j
                if(bounding_box(v(i,:),v(j,:),o_trees(k,:)))
                    % Compute the distance from k to the edge between i and j
                    u = abs(((v(j,1)-v(i,1))*(v(i,2)-o_trees(k,2)))-((v(i,1)-o_trees(k,1))*(v(j,2)-v(i,2))))/sqrt((v(j,1)-v(i,1))^2+(v(j,2)-v(i,2))^2);
                    % If it is too close, discard the edge
                    if (u < threshold || i == j)
                        edges(i,j) = 0;
                        break;
                    end
                end
            end
        end
    end
end

% Check to see if the robot is connected to the goal
[reached_goal] = vertex_connect(min_distance(v,robot),min_distance(v,goal),edges);

% Merge close edges
for i=1:n
	for j=1:n
		if((edges(i,j) == 1 || edges(j,i) == 1) && sqrt((v(i,1)-v(j,1))^2 + (v(i,2)-v(j,2))^2) < 1)
			for k=1:n
				if (edges(i,k) == 1 || edges(j,k) == 1 || edges(k,i) == 1 || edges(k,j) == 1)
					edges(i,k) = 1;
					edges(j,k) = 1;
					edges(k,i) = 1;
					edges(k,j) = 1;
				end
			end
		end
	end
end

% Select the next best vertex to navigate to
[next_v,score] = next_vertex(min_distance(v,robot),min_distance(v,goal),edges,v,1,0);

if (reached_goal == 1 && next_v ~= (min_distance(v,robot)))
    local_goal = v(next_v,:);
else
    termination_flag = 1;
end
end % End function declaration

function [in_box] = bounding_box(i,j,k)
% INPUT
% i : 2D point
% j : 2D point
% k : 2D point to be tested if inside the bounding box for i and j
% OUTPUT
% in_box : 1 if the point is inside the box, 0 otherwise
a = [i(1),i(2)];
b = [i(1),j(2)];
c = [j(1),j(2)];
d = [j(1),i(2)];

if (b(1) > d(1))
    temp = b;
    b = d;
    d = temp;
    temp = c;
    c = a;
    a = temp;
end
if (b(2) > a(2))
    temp = a;
    b = a;
    a = temp;
    temp = c;
    c = d;
    d = temp;
end

if (k(1) < c(1) && k(2) < a(2) && k(1) > a(1) && k(2) > b(2))
    in_box = 1;
else
    in_box = 0;
end
end

% Recursive depth-first search, can be made iterative if memory is prohibitive
function [path, at_goal] = depth_first_search(current,edges,goal,current_path)
% INPUT
% current : the current vertex to choose the next move from
% edges   : adjacency matrix
% goal    : index of goal node
% current_path : indices of previously explored nodes
% OUTPUT
% path : indices of nodes from initial node to goal
% at_goal : 1 if goal is reached, 0 if goal is not reached
[m,n] = size(edges);
at_goal = 0;
path = [];
for i=1:m
    if (edges(current,i) == 1 && ~any(current_path==i))
        if (i == goal)
            path = [current_path,goal];
            at_goal = 1;
            break;
        end
        [path,at_goal] = depth_first_search(i,edges,goal,[current_path,i]);
        if (at_goal == 1)
            break;
        end
    end
end
end

% Finds the minimum distance vertex from a point
function [min_vertex] = min_distance(vertices,point)
% INPUT
% vertices : xy coordinates of vertices
% point : point to measure distance to
% OUTPUT
% min_vertex : index of the vertex with minimum distance
min_vertex = 1;
min_distance = sqrt((vertices(1,1) - point(1,1))^2 + (vertices(1,2) - point(1,2))^2);
[m,n] = size(vertices);
for i=2:m
    cur_distance = sqrt((vertices(i,1) - point(1,1))^2 + (vertices(i,2) - point(1,2))^2);
    if (cur_distance < min_distance)
        min_distance = cur_distance;
        min_vertex = i;
    end
end
end

% Determine if two vertices in a graph are connected
function [connected] = vertex_connect(start,goal,edges)
% INPUT
% start : index of starting vertex
% goal  : index of goal vertex
% edges : adjacency matrix
% OUTPUT
% connected : 0 if no path exists, 1 if there is a path
connected = 0;
[m,junk] = size(edges);
reached = [start];
explored = [];
while (1)
    [junk,n] = size(reached);
    current = 0;
    for i=1:n
        if (~any(explored==reached(i)))
            current = i;
            break;
        end
    end
    if (current == 0)
        break;
    end
    for i=1:m
        if(edges(reached(current),i)==1 && ~any(explored==i))
            reached = [reached,i];
        end
    end
    for i=1:m
        if(edges(i,reached(current))==1 && ~any(explored==i))
            reached = [reached,i];
        end
    end
    if (any(reached==goal))
        connected = 1;
        break;
    end
    explored = [explored,reached(current)];
end
end

% Select the next vertex using a heuristic that the next best edge is the
% one which is in a path of length n reaching a vertex closest to the goal
function [best,best_score] = next_vertex(start,goal,edges,vertices,depth,path_length)
% INPUT
% start : index of starting vertex
% goal  : index of goal vertex
% edges : adjacency matrix
% vertices : xy coordinates of each vertex
% depth : number of vertices to search in the graph
% path_length : length of current path
% OUTPUT
% next : index of next vertex
best = start;
best_score = realmax;
threshold = 0.5;
[n,junk] = size(edges);
current_best = 0;
for i=1:n
    if (edges(start,i) == 1 || edges(i,start) == 1)
        from_start = sqrt(((vertices(i,1) - vertices(start,1))^2) + ((vertices(i,2) - vertices(start,2))^2));
        from_goal = sqrt(((vertices(i,1) - vertices(goal,1))^2) + ((vertices(i,2) - vertices(goal,2))^2));
        path_length = path_length + from_start;
        if (i == goal || from_goal < threshold)
            current_score = path_length - 100000;
        elseif (depth == 1)
            current_score = from_goal;
            
        else
            [current_best,current_score] = next_vertex(i,goal,edges,vertices,depth -1,path_length);
        end
        if (current_score < best_score && from_start > threshold)
            best = i;
            best_score = current_score;
        elseif (current_score < best_score && from_start <= threshold && current_best ~= 0)
            best = current_best;
            best_score = current_score;
        end
    end
end
end