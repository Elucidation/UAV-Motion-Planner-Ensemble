function h = plotlocal(obs, goal, path, closedist, bounds, h)
if (nargin == 4 || (nargin > 4 && isempty(bounds)))
    minx = min([path(:,1); goal(1)]);
    miny = min([path(:,2); goal(2)]);
    maxx = max([path(:,1); goal(1)]);
    maxy = max([path(:,2); goal(2)]);
    diffx = maxx - minx;
    diffy = maxy - miny;
    maxx = ceil(maxx + diffx * 1);
    maxy = ceil(maxy + diffy * 1);
    minx = floor(minx - diffx * 1);
    miny = floor(miny - diffy * 1);
else
    minx = bounds(1);
    maxx = bounds(2);
    miny = bounds(3);
    maxy = bounds(4);
end
sizex = maxx - minx + 1;
sizey = maxy - miny + 1;
cost = zeros(sizey,sizex);
for x = 1:sizex
    for y = 1:sizey
        cost(y,x) = calccost([x+minx-1 y+miny-1], obs, goal, 'linear2', closedist);
    end
end
startpos = [path(1,:) interp2((1:sizex)+minx-1, (1:sizey)+miny-1, cost, path(1,1), path(1,2))];
for i = 1:length(path(:,1))
    if (path(i,1) >= minx && path(i,1) <= maxx && path(i,2) >= miny && path(i,2) <= maxy)
        pathtoplot(i,:) = [path(i,1) path(i,2) interp2((1:sizex)+minx-1, (1:sizey)+miny-1, cost, path(i,1), path(i,2))];
    end
end
if (nargin == 6)
    figure(h)
    hold off
else
    h = figure;
end
% surf(minx:maxx, miny:maxy, cost, 'EdgeColor', 'none')
contour(minx:maxx, miny:maxy, cost, 100);
hold on
for i = 1:length(obs)
    if (obs{i}.x >= minx && obs{i}.x <= maxx && obs{i}.y >= miny && obs{i}.y <= maxy)
%         plot3(obs{i}.x,obs{i}.y,interp2((1:sizex)+minx-1,(1:sizey)+miny-1,cost,obs{i}.x,obs{i}.y)+.2,'o','MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',10);
        plot(obs{i}.x, obs{i}.y, 'o','MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',10);
    end
end
if (goal(1) >= minx && goal(1) <= maxx && goal(2) >= miny && goal(2) <= maxy)
%     plot3(goal(1),goal(2),interp2((1:sizex)+minx-1,(1:sizey)+miny-1,cost,goal(1),goal(2))+.2,'p','MarkerEdgeColor','c','MarkerFaceColor','c','MarkerSize',10);
    plot(goal(1), goal(2), 'p','MarkerEdgeColor','c','MarkerFaceColor','c','MarkerSize',10);
end
if (exist('pathtoplot', 'var'))
%     plot3(pathtoplot(:,1), pathtoplot(:,2), pathtoplot(:,3)+.2, 'k-', 'LineWidth', 5);
    plot(pathtoplot(:,1), pathtoplot(:,2), 'k-', 'LineWidth', 5);
end
if (startpos(1) >= minx && startpos(1) <= maxx && startpos(2) >= miny && startpos(2) <= maxy)
%     plot3(startpos(1),startpos(2),startpos(3)+.2,'d','MarkerEdgeColor','m','MarkerFaceColor','m','MarkerSize',10);
    plot(startpos(1), startpos(2), 'd','MarkerEdgeColor','m','MarkerFaceColor','m','MarkerSize',10);
end
% view(2)
% axis image
% pos = get(gcf, 'position');
% pos(1:2) = [100 600];
% set(gcf, 'position', pos);