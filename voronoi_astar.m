function path = voronoi_astar(start, goal, v, edges)
threshold = 0.5;
frontier = cell(1000,1);
frontierq = pq_create(1000);
explored = cell(1000, 1);
node.parents = {};
node.vert = start;
node.g = 0;
node.h = dist(start, goal, v);
node.cost = node.g + node.h;
frontier{1} = node;
pq_push(frontierq, 1, node.cost);
fidx(1) = 1;
eidx = [];
fidxnext = 2;
eidxnext = 1;
done = false;


while (~done)
	if (pq_size(frontierq) == 0)
        solution = false;
        done = true;
    else
        [i c] = pq_pop(frontierq);
        node = frontier{i};
        frontier{i} = NaN;
        fidx(find(fidx==i,1)) = [];
        if (at_goal(node.vert, goal, threshold, v))
            noparents = node;
            noparents.parents = {};
            noparents.next = 0;
            solution = [node.parents noparents];
            done = true;
        else
            if (isempty(eidx))
                eidx(1) = eidxnext;
            else
                eidx(end+1) = eidxnext;
            end
            eidxnext = eidxnext + 1;
            explored{eidx(end)} = node.vert;
            for next = unique([find(edges(node.vert,:)==1) find(edges(:,node.vert)==1)'])
                efound = false;
                ffound = 0;
                for i = eidx
                    if (next == explored{i})
                        efound = true;
                        break
                    end
                end
                for i = fidx
                    if (next == frontier{i}.vert)
                        ffound = i;
                        break
                    end
                end
                noparents = node;
                noparents.parents = {};
                noparents.next = next;
                newnode.parents = [node.parents noparents];
                newnode.vert = next;
                newnode.g = node.g + dist(node.vert, next, v);
                newnode.h = dist(next, goal, v);
                newnode.cost = newnode.g + newnode.h;
                if (~efound && ffound == 0)
                    if (isempty(fidx))
                        fidx(1) = fidxnext;
                    else
                        fidx(end+1) = fidxnext;
                    end
                    fidxnext = fidxnext + 1;
                    frontier{fidx(end)} = newnode;
                    pq_push(frontierq, fidx(end), -newnode.cost);
                elseif (ffound > 0)
                    frontier{ffound} = newnode;
                    pq_push(frontierq, ffound, -newnode.cost);
                end
            end
        end
    end
end
path = [];
if (iscell(solution))
    for i = 1:length(solution)
        path = [path solution{i}.vert];
    end
end
end


function d = dist(pos, goal, v)
d = norm(v(pos,:) - v(goal,:));
end

function atgoal = at_goal(pos, goal, threshold, v)
if (pos == goal)%(dist(pos, goal, v) < threshold)
    atgoal = 1;
else
    atgoal = 0;
end
end