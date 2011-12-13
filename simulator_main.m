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
reset(RandStream.getDefaultStream)

%% USER DEFINED Values

% Simulator
TURNS = 200; % Number of turns to simulate
N = 500; % Number of points in playing field
BBOX = [0 0 100 100]; % playing field bounding box [x1 y1 x2 y2]
global_goal = [80 70]; % Global goal position (must be in BBOX)

% Robot
robot_pos = [10 20]; % x,y position
robot_rov = 15; % Range of view
DMAX = 3; % max robot movement in one turn
MINDIST = 1; % Minimum distance from goal score

% FIGURES (LOCAL - contour, and GLOBAL - 2d trail)
DO_LOCAL = false; % plot local planner contour figure
FIG_SIZE = [560 420]*2; % Both Figure sizes
FIG_POS1 = [50 50]; % Local figure position
FIG_POS2 = [100 50]; % Global figure position

% AVI FILES
Global_filename = sprintf('simAll_2D_5_ROV%i_N%i.avi',robot_rov,N); 
Local_filename = sprintf('simAll_contour_5_ROV%i_N%i.avi',robot_rov,N);
DO_AVI = true; % write any avi files at all (Global & Local)
DO_LOCAL_AVI = false; % write contour avi file
FRAME_REPEATS = 2; % Number of times to repeat frame in avi (slower framerate below 5 which fails on avifile('fps',<5))

% Noise Function
[noiseFilt sigmaV] = noiseFilter(); %noiseFilt = makeNoiseFilter(0,0.4,30,10,1.4);
%sigmaV = @(x) 0; % Zero sigma function


%% SETUP
getRandPoints = @(N,x1,y1,x2,y2) [ones(1,N)*x1; ones(1,N)*y1] + rand(2,N).*[ones(1,N)*(x2-x1); ones(1,N)*(y2-y1)];
getDist = @(a,b) sqrt((b(2)-a(2))^2 + (b(1)-a(1))^2);
obstacles = getRandPoints(N,BBOX(1),BBOX(2),BBOX(3),BBOX(4)); % Initial random distribution of points
%load obstacle2;
%load obstacles;
% obstacles = [10 15; 
%              10 5;
%              10 10;
%              10 2;
%              10 18;]';

views = zeros(1,N); % Number of times point has been seen before, this affects the noiseFilter as value passed into viewsCount (more = less noise)
obstacleEstimate = zeros(2,N); % Last (estimated) Known position of obstacles
obstacleLastKnown = zeros(2,N); % Last known all obstacles
distances = zeros(1, N); % Init current distances between robot and obstacles
robot_trail = zeros(TURNS,2); % Initialize robot position history

% Axis & Views
AX = [BBOX(1),BBOX(3),BBOX(2),BBOX(4)];
azel = [-123.5 58]; % for view in saving files
closestEncounter = inf; % Closest distance of robot to any obstacle
closestPos = [0,0]; % Position of closest encounter

% Run loop variables
foundGoal = 0;
checkRepeat0It = 0;


% Setup AVI files
if DO_AVI
    aviobj = avifile(Global_filename,'compression','None','fps',5); % Global AVI
    if DO_LOCAL_AVI
        aviobj2 = avifile(Local_filename,'compression','None','fps',5); % Local AVI
    end
end

%% Set up Figures
if DO_LOCAL
    fh1 = figure(1); % 3D Contour Figure (Left)
    pos = get(gcf, 'position');
    pos(1:2) = FIG_POS1;
    pos(3:4) = FIG_SIZE;
    set(gcf, 'position', pos);
end

fh2 = figure(2); % Ground Truth Figure with Voronoi (Right)
axis(AX);
pos = get(fh2, 'position');
pos(1:2) = FIG_POS2;
pos(3:4) = FIG_SIZE;
set(fh2, 'position', pos);

%% RUN LOOP
fprintf(2,'  Magenta Diamond - UAV position\n');
fprintf(2,'        Blue Star - Local Goal\n');
fprintf(2,'         Red Star - Global Goal\n');
fprintf(2,'        Black dot - Ground truth positions of obstacles (unknown to UAV)\n');
fprintf(2,'        Green dot - Last known observed position of obstacle (connected to ground truth position with green dotted line)\n');
fprintf(2,'Blue dotted-lines - Voronoi diagram\n\n');

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
   
   % The noise based on view count for all obstacles
   noise = noiseFilt(  views );
   
   % update noise only for those currently seeing 
   obstacleLastKnown(:,viewCurrent~=0) = obstacles(:,viewCurrent~=0) + noise(:,viewCurrent~=0);
   
   % Just known obstacles, vs obstacleLastKnown includes those not
   % currently visible
   obstacleEstimate = obstacleLastKnown(:,views~=0);
   % Just currently visible obstacles
   obstacleCurrent = obstacleLastKnown(:,viewCurrent~=0);
   
   obstacleObjects = cell(1,sum(viewCurrent));
   for k = 1:sum(viewCurrent)
       obstacleObjects{k}.x = obstacleCurrent(1,k);
       obstacleObjects{k}.y = obstacleCurrent(2,k);
       %v = views(views~=0);
       v = views(logical(viewCurrent));
       obstacleObjects{k}.sig = sigmaV(v(k));
   end
   
   
   
   %% VORONOI PLANNER - Determine Local Goal given known obstacles, and global goal
   % obstacleEstimate' has every obstacle ever seen
   [local_goal,noPathExists,VX,VY] = voronoi_planner(obstacleEstimate', robot_pos, global_goal);
%    local_goal = robot_pos+20*(global_goal-robot_pos)/getDist(robot_pos,global_goal); % 10% towards global_goal
   
   % If Voronoi is being bad, ignore it
   if (getDist(robot_pos,local_goal) > getDist(robot_pos,global_goal))
       local_goal = global_goal; % global goal is closer than local goal
   end
   
   % Only given those obstacles it currently sees
   local_goal_path = localplan(robot_pos, local_goal, obstacleObjects);
   %local_goal_path = localplan(robot_pos, global_goal, obstacleObjects);
   if size(local_goal_path,1) == 1
       checkRepeat0It = checkRepeat0It + 1;
       if (checkRepeat0It > 10)
           % Stuck in equilibrium, apply nudge
           local_goal_path = [local_goal_path;local_goal];
           checkRepeat0It = 0;
       end
   end
   
   
   %% Update Robot
   d = 0; k = 0;
   % Update Robot position along local path trail for distance DMAX
   while d < DMAX
       robot_trail(i,:) = robot_pos; % Keep history of robot positions;
       if getDist(robot_pos,global_goal) < MINDIST % Found the goal!
           fprintf('You made it in %i steps.\n',i);
           foundGoal = 1;
           robot_pos = global_goal;
           break
       elseif getDist(robot_pos,global_goal) < (DMAX-d) % Goal within DMAX-d!
           fprintf('You made it in %i steps.\n',i);
           foundGoal = 1;
           robot_pos = global_goal;
           break
       end
       
       if (2+k > size(local_goal_path,1)) % Next path point is further than can walk
           % Can't go any further so just go to the last point
           next_pos = local_goal_path(end,:); % choose last position
           robot_pos = next_pos; % Update position
           break;
       end
       % Next position is point on local goal path
       next_pos = local_goal_path(2+k,:); % choose next position from local plan
       d = d + getDist(robot_pos,next_pos); % distance traveled increased by motion
       k = k + 1; % next point in path
       % Update position 
       if (d > DMAX)
           % robot wants to move too far, so interpolate along distance
           robot_pos = robot_pos + DMAX*(next_pos-robot_pos)/getDist(robot_pos,next_pos);
       else
          robot_pos = next_pos; 
       end
   end
   
   %% Find closest encounter to ground truth obstacles
   d = 0;
   for k = 1:N
       d = getDist(robot_pos,obstacles(:,k));
       if d < closestEncounter
           closestEncounter = d;
           closestPos = robot_pos;
       end
   end
   
   %% GRAPHICS -----------------------------------------
   %% What we observe ( RIGHT FIGURE)
   figure(fh2);
   % Plot VORONOI LINES
   hold off;
   plot(VX,VY,'b:');   
   set(fh2(1:end-1),'xliminclude','off','yliminclude','off'); % keep infinite lines clipped
   hold on;
   axis('equal'); axis(AX);
   
   % Observed Obstacles
   observedObsH = scatter(obstacleEstimate(1,:),obstacleEstimate(2,:),'g','filled');
   
   hold on;
   % Ground truth - black
   groundTruthH = scatter( obstacles(1,:),obstacles(2,:),'k.');
   
   % Plot connector line between ground truth to observed
   gt = obstacles(:,views~=0);
   es = obstacleEstimate;
   for k = 1:size(es,2)
       % For each connection (estimate)
       plot([gt(1,k) es(1,k)],[gt(2,k) es(2,k)],'g:');
   end
   
   % Robot
   plot([robot_trail(robot_trail(:,1)~=0,1); robot_pos(1)],[robot_trail(robot_trail(:,2)~=0,2); robot_pos(2)],'b.-'); % Robot Trail
   robotPosH = scatter(robot_pos(1), robot_pos(2),80, 'md','filled'); % Robot position
   drawCircle(robot_pos(1),robot_pos(2), robot_rov, 'b-'); % Robot Field of View (vision range)
   
   % Local & Global Goals
   localGoalH = scatter(local_goal(1),local_goal(2),100,'cp','filled'); % Plot local goal
   globalGoalH = scatter(global_goal(1),global_goal(2),150,'rp','filled'); % Plot global goal 
   title(sprintf('%i Obstacles, Turn %i : Distance to Global Goal = %g\nClosest Encounter = %g @ P(%g,%g)',N,i,getDist(robot_pos,global_goal),closestEncounter,closestPos(1),closestPos(2)));
   legend([robotPosH localGoalH globalGoalH observedObsH groundTruthH],'UAV position','Local Goal','Global Goal', 'Observed/Estimated Obstacle Positions', 'Ground Truth Obstacle Positions','Location','NorthWest');
   hold off;
   % write AVI of GLOBAL
   if DO_AVI
       f2 = getframe(fh2);
       for k = 1:FRAME_REPEATS
           aviobj = addframe(aviobj,f2); % Save to avi
       end
   end
   
   
   %% Plot 3D Contour path for LOCAL Planner (ie. to Local goal)
   if DO_LOCAL
       plotlocal(obstacleObjects, local_goal, local_goal_path, AX ,fh1); % new figure 3d contour
       axis(AX);
       view(azel);
       view(2);

       % SAVE FIG AND AVI
       %saveas(gcf,sprintf('localMinima%i',i),'fig');
       title(sprintf('Turn %i : Distance to Local Goal = %g',i,getDist(robot_pos,global_goal)));
       if DO_LOCAL_AVI
           f1 = getframe(fh1);
           for k=1:FRAME_REPEATS
               aviobj2 = addframe(aviobj2,f1); % Save to avi
           end
       end
   end
   
   %% END GRAPHICS ---------------------
   
   %% Check End State or Stuck State
   if (foundGoal == 1) % If we found goal, we're done!
       break
   end
   if i>10&&(sum(std(robot_trail(i-10:i,:)).^2) < 0.3) % If last 10 points were roughly same spot, we're stuck and done.
       disp('Stuck in Loop.')
       break
   end
   pause(0.0001);
end
if DO_AVI
    aviobj = close(aviobj);
    if DO_LOCAL_AVI
        aviobj2 = close(aviobj2);
    end
end