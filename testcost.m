tic
RandStream.setDefaultStream(RandStream('mt19937ar', 'Seed', ceil(toc*1000000)*0+35));
clear
close all

mapsizex = 200;
mapsizey = 100;
obs = cell(1,randi(20));
for i = 1:length(obs)
    obs{i}.x = randi(mapsizex);
    obs{i}.y = randi(mapsizey);
    obs{i}.sig = rand * 20;
end
goal = [randi(mapsizex) randi(mapsizey)];
cost = zeros(mapsizey,mapsizex);
for x = -50:(mapsizex+50)
    for y = -50:(mapsizey+50)
        cost(y+51,x+51) = calccost([x y], obs, goal, 'linear2');
    end
end
f = figure;
surfc(-50:(mapsizex+50), -50:(mapsizey+50), cost, 'EdgeColor', 'none')
hold on
for i = 1:length(obs)
    plot3(obs{i}.x,obs{i}.y,cost(obs{i}.y+51,obs{i}.x+51)+.2,'o','MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',10);
end
plot3(goal(1),goal(2),cost(goal(2)+51,goal(1)+51)+.2,'s','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10);
x = randi(mapsizex);
y = randi(mapsizey);
tic
[path slope] = localplan([x y], goal, obs);
toc
for i = 1:length(path(:,1))
    path(i,3) = interp2(-50:(mapsizex+50), -50:(mapsizey+50), cost, path(i,1), path(i,2));
end
figure(f);
plot3(path(:,1), path(:,2), path(:,3)+.2, 'k-', 'LineWidth', 5);
view(2)
% axis image