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
[noiseFilt sigmaV] = noiseFilter(); %noiseFilt = makeNoiseFilter(0,0.4,30,10,1.4);
%sigmaV = @(x) 0; % Zero sigma function

TURNS = 100; % Number of turns to simulate
N = 5; % Number of points in playing field

BBOX = [0 0 20 20]; % playing field bounding box [x1 y1 x2 y2]
%ax = [0 100 0 100 0 100];
ax = [0 20 0 20];
azel = [-123.5 58]; % for view in saving files


global_goal = [18 10]; % Change this if BBOX changes

robot_pos = [2 10]; % x,y position
robot_rov = 40; % Range of view
DMAX = 1; % max robot movement in one turn
MINDIST=2; % Minimum distance from goal score

%% SETUP
getRandPoints = @(N,x1,y1,x2,y2) [ones(1,N)*x1; ones(1,N)*y1] + rand(2,N).*[ones(1,N)*(x2-x1); ones(1,N)*(y2-y1)];
getDist = @(a,b) sqrt((b(2)-a(2))^2 + (b(1)-a(1))^2);
%obstacles = getRandPoints(N,BBOX(1),BBOX(2),BBOX(3),BBOX(4)); % Initial random distribution of points
%load obstacles;
obstacles = [10 15; 
             10 5;
             10 10;
             10 2;
             10 18;]';

views = zeros(1,N); % Number of times point has been seen before, this affects the noiseFilter as value passed into viewsCount (more = less noise)
obstacleEstimate = zeros(2,N); % Last (estimated) Known position of obstacles
%obstacleCurrent = zeros(2,N); % Currently known obstacle positions
obstacleUncertainty = zeros(2,N); % Last (estimated) Known Error of points
distances = zeros(1, N); % Init current distances between robot and obstacles
robot_trail = zeros(TURNS,2); % Initialize robot position history

%% RUN LOOP
fprintf(2,'Black stars - Points exist but not currently visible\n');
fprintf(2,'       Red X - Visible Points, Actual location\n');
fprintf(2,'Blue circles - Visible Points, Noisy Location\n\n');

%aviobj = avifile('sim3d_T20_local_minima.avi','compression','None');

foundGoal = 0;
%% MAIN LOOP -----------------------------------------------
for i = 1:TURNS
   fprintf('TURN %i : ',i);
   viewCurrent = zeros(1,N); % localPlanner view sees only currently visible obstacles
   
   for j=1:N
       distances(j) = getDist(robot_pos,obstacles(:,j));
       if (distances(j) < robot_rov)
            views(j) = views(j) + 1;
            viewCurrent(j) = 1; % only current planner sees this
            
%        else
%            views(j) = 0;
       end
   end
   
   noise = noiseFilt(  views );
   obstacleEstimate = obstacles(:,views~=0) + noise(:,views~=0);
   obstacleCurrent = obstacles(:,viewCurrent~=0) + noise(:,viewCurrent~=0);
   %obsKnown = objectsLastKnown(:,views~=0);
   obstacleUncertainty = noise;
   
   obstacleObjects = cell(1,sum(viewCurrent));
   for k = 1:sum(viewCurrent)
       obstacleObjects{k}.x = obstacleCurrent(1,k);
       obstacleObjects{k}.y = obstacleCurrent(2,k);
       v = views(views~=0);
       obstacleObjects{k}.sig = sigmaV(v(k));
   end
   
%    obstacleObjects = cell(1,size(objectsLastKnown(1,views~=0),2));
%    for k = 1:size(obsKnown,2)
%        obstacleObjects{k}.x = obsKnown(1,k);
%        obstacleObjects{k}.y = obsKnown(2,k);
%        v = views(views~=0);
%        obstacleObjects{k}.sig = sigmaV(v(k));
%    end
   
   %% VORONOI GRAPHICS FORCED TO BE HERE :(
   % Figure out local plan potential field
   figure(2); % Ground Truth Figure with Voronoi (Right)
   hold off;
   % obstacleEstimate' has every obstacle ever seen
   local_goal = voronoi_planner(obstacleEstimate', robot_pos, global_goal);
%    local_goal = robot_pos+20*(global_goal-robot_pos)/getDist(robot_pos,global_goal); % 10% towards global_goal
%    if (getDist(robot_pos,local_goal) > getDist(robot_pos,global_goal))
%        local_goal = global_goal; % global goal is closer than local goal
%    end
   % Only given those obstacles it currently sees
   local_goal_path = localplan(robot_pos, local_goal, obstacleObjects);
   %local_goal_path = localplan(robot_pos, global_goal, obstacleObjects);
   
   
   %% Update Robot
   d = 0;
   k = 0;
   while d < DMAX
       robot_trail(i,:) = robot_pos; % Keep history of robot positions;
       if getDist(robot_pos,global_goal) < MINDIST
           % Found the goal!
           fprintf('You made it in %i steps.\n',i);
           foundGoal = 1;
           break
       end
       if (2+k > size(local_goal_path,1)) 
           % Can't go any further so just go to the last point
           next_pos = local_goal_path(end,:); % choose last position
           robot_pos = next_pos; % Update position
           break;
       end
       next_pos = local_goal_path(2+k,:); % choose next/2nd position from local plan
       d = d + getDist(robot_pos,next_pos);
       k = k + 1;
       % Update position 
       if (d > DMAX)
           % robot wants to move too far, so interpolate along distance
           robot_pos = robot_pos + DMAX*(next_pos-robot_pos)/getDist(robot_pos,next_pos);
       else
          robot_pos = next_pos; 
       end
   end

   %Ground truth
%    figure(1);   
%    axis([BBOX(1),BBOX(3),BBOX(2),BBOX(4)]);
%    scatter(robot_pos(1), robot_pos(2), 'r');
%    hold on;
%    scatter( obstacles(1,views==0),obstacles(2,views==0),'g');
%    scatter( obstacles(1,views==1),obstacles(2,views==1),'b');
%    scatter(obstacleEstimate(1,:),obstacleEstimate(2,:),'m');
%    hold off;
   
   
   %% GRAPHICS -----------------------------------------
   
   %% What we observe ( RIGHT FIGURE)
   figure(2); % Ground Truth Figure with Voronoi (Right)
   hold on;
   pos = get(gcf, 'position');
   pos(1:2) = [720 278];
   set(gcf, 'position', pos);
   axis([BBOX(1),BBOX(3),BBOX(2),BBOX(4)]);  
   
   % What we observe
   scatter( obstacleEstimate(1,:),obstacleEstimate(2,:),'g','filled');
   
%    % Last known observed positions
%    scatter( objectsLastKnown(1,views~=0), objectsLastKnown(2,views~=0), 'g');
%    
%    % Plot Last known connector lines between ground truth to observed
%    gt = obstacles(:,views~=0);
%    es = obsKnown;
%    for k = 1:size(es,2)
%        % For each connection (estimate)
%        plot([gt(1,k) es(1,k)],[gt(2,k) es(2,k)],'g:');
%    end
   
   hold on;
   % Ground truth - black
   scatter( obstacles(1,:),obstacles(2,:),'k.');
   
   % Plot connector line between ground truth to observed
   gt = obstacles(:,views~=0);
   es = obstacleEstimate;
   for k = 1:size(es,2)
       % For each connection (estimate)
       plot([gt(1,k) es(1,k)],[gt(2,k) es(2,k)],'g:');
   end
   
   % Plot robot trail
   plot([robot_trail(robot_trail(:,1)~=0,1); robot_pos(1)],...
        [robot_trail(robot_trail(:,2)~=0,2); robot_pos(2)],'b.-'); % Show robot position trail
   
    % Plot robot position
   scatter(robot_pos(1), robot_pos(2),80, 'md','filled');
   
   % Plot local goal
   scatter(local_goal(1),local_goal(2),100,'cp','filled'); % Final goal plot
   
   % Plot global goal 
   scatter(global_goal(1),global_goal(2),150,'rp','filled'); % Final goal plot
   
   axis([BBOX(1),BBOX(3),BBOX(2),BBOX(4)]);
   %axis([BBOX(1),BBOX(3),BBOX(2),BBOX(4)]*2-50);
   hold off;
   
   
   % Plot 3D Contour path
   fh1 = figure(1); % 3D Contour Figure (Left)
   plotlocal(obstacleObjects,local_goal,local_goal_path, [BBOX(1),BBOX(3),BBOX(2),BBOX(4)],fh1); % new figure 3d contour
   %plotlocal(obstacleObjects,global_goal,local_goal_path, [BBOX(1),BBOX(3),BBOX(2),BBOX(4)],fh1); % new figure 3d contour
   pos = get(gcf, 'position');
   pos(1:2) = [160 278];
   set(gcf, 'position', pos);
   axis(ax);
   %view(azel);
   view(2);
   
   % SAVE FIG AND AVI
   %saveas(gcf,sprintf('localMinima%i',i),'fig');
   %aviobj = addframe(aviobj,getframe(gcf)); % Save to avi
   
   %% END GRAPHICS ---------------------
   
   
   
   title(sprintf('Turn %i/%i : Distance to Goal = %g',i,N,getDist(robot_pos,global_goal)));
   fprintf('END TURN %i\n',i);
   %pause
   if (foundGoal == 1)
       break
   end
   pause(0.0001);
   %pause
end
%aviobj = close(aviobj);
