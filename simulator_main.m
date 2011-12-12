close all; clear all; clc;
% What this file does:
% Sets up a field
% Black stars are points that exist but aren't currently visible
% Red X's are points that are visible, that is their 'actual' location, but not what we see
% Blue circles are points that are visible, that is the location we see after noise is added

% It also uses getVisible.m to get whether angles are in field of view

% Right now the noise function only takes into account number of times point has previously been viewed
% It would be interesting to make noise also a function of distance, which can be implemented easily
% Any noise filter you'd like to implement should take an output of the format:
% 2xN for N visible points, then it can just directly replace the current noiseFilter


%% USER DEFINED Values
N = 100; % Number of points in playing field
BBOX = [0 0 1000 1000]; % playing field bounding box [x1 y1 x2 y2]
noiseFilt = @noiseFilter; %noiseFilt = makeNoiseFilter(0,0.4,30,10,1.4);
robot_pos = [60 40]'; % x,y position
robot_rov = 200;
global_goal = [900 800]';


%% SETUP
getRandPoints = @(N,x1,y1,x2,y2) [ones(1,N)*x1; ones(1,N)*y1] + rand(2,N).*[ones(1,N)*(x2-x1); ones(1,N)*(y2-y1)];
getDist = @(a,b) sqrt((b(2)-a(2))^2 + (b(1)-a(1))^2);
obstacles = getRandPoints(N,BBOX(1),BBOX(2),BBOX(3),BBOX(4)); % Initial random distribution of points

views = zeros(1,N); % Number of times point has been seen before, this affects the noiseFilter as value passed into viewsCount (more = less noise)
obstacleEstimate = zeros(2,N); % Last (estimated) Known position of points
obstacleUncertainty = zeros(2,N); % Last (estimated) Known Error of points
distances = zeros(1, N);

%% RUN LOOP
fprintf(2,'Black stars - Points exist but not currently visible\n');
fprintf(2,'       Red X - Visible Points, Actual location\n');
fprintf(2,'Blue circles - Visible Points, Noisy Location\n\n');

TURNS =20;

%if goal reached
for i = 1:TURNS
   for j=1:N
       distances(j) = getDist(robot_pos,obstacles(:,j));
       if (distances(j) < robot_rov)
            views(j) = views(j) + 1;
       else
           views(j) = 0;
       end
   end
   
   %Uncertainty update
   for j=1:N
       if views(j) > 0
           noise = noiseFilt(  views(j) );
           actual = obstacles(:,j);
           obstacleEstimate(:,j) = actual + noise;
           obstacleUncertainty(:,j) = noise;
       end
   end

%    local_goal = Will(robot_pos, obstacles, global_goal)
%    Billy(robot_pos, obstacles-sigma,pos, local_goal)
%    look into the sigma thing (maybe it's just noise?)
%    change noise thing to proper gaussian

   % Update Robot

   %Ground truth
   figure(1);
   clf;
   hold on;
   axis([BBOX(1),BBOX(3),BBOX(2),BBOX(4)]);
   
   scatter(robot_pos(1), robot_pos(2), 'r');
   for j=1:N
       if (views(j)==0)
           scatter(obstacles(1,j),obstacles(2,j),'g');
       else
           scatter(obstacles(1,j),obstacles(2,j),'b');
       end
       
       scatter(obstacleEstimate(1,j),obstacleEstimate(2,j),'m');
   end
   
   % What we observe
   figure(2);
   clf;
   hold on;
   axis([BBOX(1),BBOX(3),BBOX(2),BBOX(4)]);
   scatter(robot_pos(1), robot_pos(2), 'r');
   for j=1:N
       if (views(j)==0)
           scatter(obstacleEstimate(1,j),obstacleEstimate(2,j),'g');
       else
           scatter(obstacleEstimate(1,j),obstacleEstimate(2,j),'b');
       end
   end
   
end
