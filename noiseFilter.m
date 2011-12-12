function [filterV sigmaV] = noiseFilter()
% Returns 2 x numPoints noise values
% where the noise variance is dependent on the values of viewsCount
% Made it slightly exponential
% OUTPUT
%   noise filter function using these values (func of viewCount
%   sigma function for this noise filter (func of viewCount)

%%%%%%%%
% NOISE FILTER
%%%%%%%%
% Gaussian noise, mean = 0, variance = 1
NOISE_MEAN = 0;
% View of 1 = variance of MAX_VARIANCE
MAX_VARIANCE = 10; % Worst possible error in x,y values (not distance, worst dist is sqrt(2*MAX_VARIANCE^2)
MIN_VARIANCE = 0; % Best possible error in x,y values
POWR = 1.4; % Exponential factor (1+, higher equals faster noise loss)
PERFECT_VIEW = 30; % At how many views till no noise

PERFECT_VIEW = nthroot(PERFECT_VIEW,POWR); % This accounts for powers

%variance = MAX_VARIANCE*max(MIN_V)ARIANCE,PERFECT_VIEW-viewsCount.^POWR+1)/PERFECT_VIEW; % MAX_VARIANCE -> 0 sigma = variance;
sigmaV = @(viewsCount) MAX_VARIANCE*max(MIN_VARIANCE,PERFECT_VIEW-viewsCount.^POWR+1)/PERFECT_VIEW;
% views must be vector of length numPoints
filterV = @(viewsCount) NOISE_MEAN + sigmaV([viewsCount;viewsCount]).*randn(2,size(viewsCount,2)); 

end