function noisyVals = noiseFilter(viewsCount)
% Returns 2 x numPoints noise values
% where the noise variance is dependent on the values of viewsCount
% Made it slightly exponential

%%%%%%%%
% NOISE FILTER
%%%%%%%%
% Gaussian noise, mean = 0, variance = 1
NOISE_MEAN = 0;
% View of 1 = variance of MAX_VARIANCE
MAX_VARIANCE = 20; % Worst possible error in x,y values (not distance, worst dist is sqrt(2*MAX_VARIANCE^2)
MIN_VARIANCE = 0.1; % Best possible error in x,y values
POWR = 1.4; % Exponential factor (1+, higher equals faster noise loss)
PERFECT_VIEW = 30; % At how many views till no noise

PERFECT_VIEW = nthroot(PERFECT_VIEW,POWR); % This accounts for powers

variance = MAX_VARIANCE*max(MIN_VARIANCE,PERFECT_VIEW-viewsCount.^POWR+1)/PERFECT_VIEW; % MAX_VARIANCE -> 0
% views must be vector of length numPoints
noisyVals = NOISE_MEAN + [variance; variance].*randn(2,size(viewsCount,2)); 

end