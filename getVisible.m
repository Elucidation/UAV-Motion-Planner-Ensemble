function visibles = getVisible(a,fov,angles)
% Finds visible angles for a given angle & fov
% Only works for angles that don't cross -180 degree mark
%getVisible = @(a,fov,angles) angles < (a + fov/2) & angles > a - (fov/2);
if (a + fov/2) > 180 % If positive fov crosses +- 180 mark
    angles = mod(angles,360); % Make angles 0 to 360
elseif (a - fov/2) < -180 % If negative fov crosses +- 180 mark
    angles = mod(angles,-360); % Make angles 0 to -360
end
visibles = angles < (a + fov/2) & angles > (a - fov/2);