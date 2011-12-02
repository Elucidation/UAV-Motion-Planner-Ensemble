function specificFilter = makeNoiseFilter(NOISE_MEAN,MIN_VARIANCE,MAX_VARIANCE,PERFECT_VIEW,POWR)
% This function returns a specific Noise filter function based on input
% Gaussian noise, mean = 0, variance = 1
%NOISE_MEAN = 0;
% View of 1 = variance of MAX_VARIANCE
%MAX_VARIANCE = 20; % Worst possible error in x,y values (not distance, worst dist is sqrt(2*MAX_VARIANCE^2)
%MIN_VARIANCE = 0.1; % Best possible error in x,y values
%POWR = 1.4; % Exponential factor (1+, higher equals faster noise loss)
%PERFECT_VIEW = 30; % At how many views till no noise

PERFECT_VIEW = nthroot(PERFECT_VIEW,POWR); % This accounts for powers
v = @(viewCountVector) MAX_VARIANCE*max(MIN_VARIANCE,PERFECT_VIEW-viewCountVector.^POWR+1)/PERFECT_VIEW;

specificFilter = @(viewCountVector) NOISE_MEAN + [v(viewCountVector); v(viewCountVector)].*randn(2,size(viewCountVector,2));
end