function delta = aim_angdiff(x, y)
%ANGDIFF Calculate difference between two angles
%   DELTA = ANGDIFF(X,Y) returns the angular difference
%   DELTA = Y-X. X and Y must be numeric arrays of the same size.
%
%   DELTA = ANGDIFF(X) returns the angular difference of adjacent elements
%   in X along the first array dimension whose size does not equal 1.
%    - If X is a numeric vector of length n, then DELTA = diff(X) returns a 
%      vector of length n-1. The elements of DELTA are the angular differences 
%      between adjacent elements in X: [X(2)-X(1) X(3)-X(2) ... X(n)-X(n-1)].
%    - If X is a nonempty, nonvector p-by-m matrix, then DELTA = diff(X) 
%      returns a matrix of size (p-1)-by-m, whose elements are the angular
%      differences between the rows of X: 
%      [X(2,:)-X(1,:); X(3,:)-X(2,:); X(p,:)-X(p-1,:)]
%
%   All angles in the resulting vector, DELTA, will be wrapped to the
%   interval (-pi,pi].
%
%   Angles are measured counter-clockwise around the positive z
%   axis, with the zero angle along the x axis. All angles are assumed to
%   be in radians.
%
%   Example:
%      % Calculate difference between two angles
%      ANGDIFF(0, 2*pi)
%      ANGDIFF(pi, -0.9*pi)
%
%      % Calculate difference between angles in vector
%      angles = [pi/4 pi/2 0.2]
%      ANGDIFF(angles)

%   Copyright 2014-2015 The MathWorks, Inc.

%#codegen

% Only allow 1 or 2 arguments
narginchk(1, 2);

if nargin == 1
    % Single input means that x should be a numeric vector
    % Syntax: ANGDIFF(X)
    
    validateattributes(x, {'single','double'}, {}, 'angdiff', 'x');       
    delta = angdiff(x);
else
    % There are two inputs
    % Syntax: ANGDIFF(X,Y)
    
    validateattributes(x, {'single','double'}, {}, 'angdiff', 'x'); 
    validateattributes(y, {'single','double'}, {'size', size(x)}, 'angdiff', 'y');    
    delta = internal_angdiff(x, y);
end
end
