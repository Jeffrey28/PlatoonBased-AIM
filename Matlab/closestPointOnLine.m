function [closestPoint, distance] = closestPointOnLine(pt1, pt2, refPt)
%This function is for internal use only. It may be removed in the future.

%closestPointOnLine Find a point closest to refPt (i.e. projection point) on line
% segment between pt1 and pt2. If the projected point is outside line
% segment then the closest vertex is returned. This function is used in
% robotics.algs.internal.PurePursuitBase class.

%   Copyright 2016 The MathWorks, Inc.

%#codegen

% Don't do any computation if points are equal
if isequal(pt1, pt2)
    closestPoint = pt1;
    distance = norm(refPt-closestPoint);
    return;
end

% Vector from pt1 to pt2
v12 = pt2 - pt1;
% Vector from refPt to pt2
vr2 = pt2 - refPt;

% Projection of the vr2 on v12, normalized by norm(v12)
alpha = v12*vr2'/(v12*v12');

% Find the closet point by interpolation
if alpha > 1
    closestPoint = pt1;
elseif alpha < 0
    closestPoint = pt2;
else
    closestPoint = alpha.*pt1 + (1-alpha).*pt2;
end

distance = norm(refPt-closestPoint);

end