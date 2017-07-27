function thetaWrap = wrapTo2Pi(theta)
%This function is for internal use only. It may be removed in the future.

%wrapTo2Pi Wrap angle in radians to interval [0 2*pi]
%
%   THETAWRAP = wrapTo2Pi(THETA) wraps angles in THETA to the interval
%   [0 2*pi]. Positive multiples of 2*pi map to 2*pi and negative
%   multiples of 2*pi map to 0.

% Copyright 2015 The MathWorks, Inc.

%#codegen

twoPiVal = cast(2*pi,'like',theta);

% Check if inputs are positive
pos = (theta > 0);

% Wrap to 2*pi
thetaWrap = mod(theta, twoPiVal);

% Make sure that positive multiples of 2*pi map to 2*pi
thetaWrap((thetaWrap == 0) & pos) = twoPiVal;

end