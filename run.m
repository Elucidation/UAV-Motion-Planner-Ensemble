clear all
close all

mapsizex = 200;
mapsizey = 100;
obs = cell(1,randi(20));
for i = 1:length(obs)
    obs{i}.A = 50;
    obs{i}.x = randi(mapsizex);
    obs{i}.y = randi(mapsizey);
    obs{i}.sig = rand * 50;
end
goal = [randi(mapsizex) randi(mapsizey)];
cost = zeros(mapsizey,mapsizex);
for x = 1:mapsizex
    for y = 1:mapsizey
        cost(y,x) = calccost([x y], obs, goal, 'linear');
    end
end
surfc(cost)
hold on
for i = 1:length(obs)
    plot3(obs{i}.x,obs{i}.y,cost(obs{i}.y,obs{i}.x)+2,'o','MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',10);
end
plot3(goal(1),goal(2),cost(goal(2),goal(1))+2,'s','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10);

