close all; clear all; clc;
% What this file does:
% Sets up a field
% Black stars are points that exist but aren't currently visible
% Red X's are points that are visible, that is their 'actual' location, but not what we see
% Blue circles are points that are visible, that is the location we see after noise is added

% Right now the noise function only takes into account number of times point has previously been viewed
% It would be interesting to make noise also a function of distance, which can be implemented easily
% Any noise filter you'd like to implement should take an output of the format:
% 2xN for N visible points, then it can just directly replace the current noiseFilter

%%%%%%%%
% USER DEFINED Values
%%%%%%%%
N = 100; % Number of points in playing field
BBOX = [0 0 100 100]; % playing field bounding box [x1 y1 x2 y2]
TURNS = 100; % Number of turns/iterations to run simulation
FPS = 30; % Frames per second update speed, doesn't really matter since matlab will go at a max speed of like 15 fps anyway

% UAV initial values
uav.pos = [40 60]'; % x,y position
uav.angle = -20; % degrees, 0 degrees is facing along +x axis
uav.fov = 90; % degrees, total field of view, 90 degrees is from angle-45 degrees to angle+45 degrees
ROT_SPEED = -1; % rotation speed per turn for uav, in degrees, 0 = no motion
%%%%%%%%


%%%%%%%%
% NOISE FILTER
%%%%%%%%
% Gaussian noise, zero mean and unit variance
% As numViews increases, noise reaches zero exponentially
noiseFilter = @(viewsCount,numPoints) randn(2,numPoints)./(1 + 0.1*[viewsCount; viewsCount].^1.4); % views must be vector of length numPoints


%%%%%%%%
% SETUP
%%%%%%%%
% Returns a matrix of 2xN x,y points inside bounding box (x1,y1) to (x2,y2)
% [x x x x x;
%  y y y y y]
% points = [x1 y1]' + rand*[x2-x1 y2-y1]'
getRandPoints = @(N,x1,y1,x2,y2) [ones(1,N)*x1; ones(1,N)*y1] + rand(2,N).*[ones(1,N)*(x2-x1); ones(1,N)*(y2-y1)];



points = getRandPoints(N,BBOX(1),BBOX(2),BBOX(3),BBOX(4)); % Initial random distribution of points
views = zeros(1,N); % Number of times point has been seen before, this affects the noiseFilter as value passed into viewsCount (more = less noise)

fprintf(stderr,'Black stars - Points exist but not currently visible\n');
fprintf(stderr,'       Red X - Visible Points, Actual location\n');
fprintf(stderr,'Blue circles - Visible Points, Noisy Location\n\n');


%%%%%%%%
% RUN LOOP
%%%%%%%%
for i = 1:TURNS
   % Returns angle between two points a = [x,y], b = [x2,y2] in degrees
   getAngle = @(a,b) atan2( b(2,:)-a(2,:) , b(1,:)-a(1,:) ) * 180/pi;

   % Based on position and orientation, find points visible
   angles = getAngle(uav.pos,points);
   isVisible = angles < (uav.angle + uav.fov/2) & angles > uav.angle - (uav.fov/2);
   visiblePoints = points( :,  isVisible );
   views(isVisible) += 1; % This keeps count of how often this point has been viewed, affecting the noise when seen (more views ~= less noise)

   whatYouSee = points(:,isVisible) + noiseFilter(  views(isVisible),  size(points(isVisible),2)  );
   hold off;
   plot(points(1,isVisible),points(2,isVisible),'rx'); % Plot all actual points in their real position
   hold on;
   plot(points(1,~isVisible),points(2,~isVisible),'k*'); % Plot all nonvisible points as black
   plot(whatYouSee(1,:),whatYouSee(2,:),'bo'); % Plot visible points at possible locations (with noise)
   plot(uav.pos(1),uav.pos(2),'ko'); % UAV position
   plot([ uav.pos(1) uav.pos(1)+10*cosd(uav.angle + uav.fov/2) uav.pos(1)+10*cosd(uav.angle + -uav.fov/2) uav.pos(1)] , ...
         [ uav.pos(2) uav.pos(2)+10*sind(uav.angle + uav.fov/2) uav.pos(2)+10*sind(uav.angle + -uav.fov/2) uav.pos(2)], 'g-') % UAV FOV lines
   axis( [ BBOX(1) BBOX(3) BBOX(2) BBOX(4) ],'equal'); % Order [x1 x2 y1 y2]
   hold off;
   
   % Statistics
   error =  sum( abs(whatYouSee - points(:,isVisible)).^2 , 2) ; % meanSquared
   fprintf(stderr, 'Mean Squared Error for visible points: %g\n',mean(error) );
   title(sprintf('Turn %i/%i : Errors Min:%.2g Max:%.2g Mean:%.2g STD:%.2g',i,TURNS, min(error),max(error),mean(error),std(error) ) );
   
   uav.angle += ROT_SPEED;
   
   pause(1.0/FPS)
end