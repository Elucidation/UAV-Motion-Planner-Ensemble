tic;pause(rand)
RandStream.setGlobalStream(RandStream('mt19937ar', 'Seed', ceil(toc*1000000)*0+318264));
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
x = randi(mapsizex);
y = randi(mapsizey);
tic
[path slope] = localplan([x y], goal, obs);
toc
plotlocal(obs, goal, path)