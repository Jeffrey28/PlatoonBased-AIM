classdef PurePursuitBase < matlab.System
    %This class is for internal use only. It may be removed in the future.
    
    %PUREPURSUITBASE Contains implementation of pure pursuit algorithm
    %   The pure pursuit controller is a geometric controller for following
    %   a path. Given a set of waypoints, the pure pursuit controller
    %   computes linear and angular velocity control inputs for a given
    %   pose of a differential drive robot.
    %
    %   PP = robotics.algs.internal.PUREPURSUITBASE returns a pure pursuit
    %   system object, PP, that computes linear and angular velocity inputs
    %   for a differential drive robot using the PurePursuit algorithm.
    %
    %   [V, W] = stepInternal(PP, POSE, WAYPTS) finds the linear velocity, V, and the
    %   angular velocity, W, for a 3-by-1 input vector POSE and an N-by-2
    %   input vector WAYPTS using the pure pursuit algorithm. The POSE is the
    %   current position, [x y orientation] of the robot. The WAYPTS are
    %   waypoints to be followed, [x y] locations. The output velocities V and W can
    %   be applied to a real or simulated differential drive robot to drive
    %   it along the desired waypoint sequence.
    %
    %   The WAYPTS can contain NaN values. The NaN values will be
    %   ignored. If all WAYPTS are NaN, then zero velocity output will be
    %   returned. If there is only one waypoint, then the output velocities
    %   will drive the robot towards that point.
    %
    %   PUREPURSUITBASE methods:
    %
    %   stepImpl    - Compute linear and angular velocity control commands
    %   info        - Get additional information about the object
    %
    %   PUREPURSUITBASE properties:
    %
    %   MaxAngularVelocity      - Desired maximum angular velocity
    %   LookaheadDistance       - Lookahead distance to compute controls
    %   DesiredLinearVelocity   - Desired constant linear velocity
    %
    %   Example
    %
    %       % Create a pure pursuit object
    %       pp = robotics.algs.internal.PurePursuit;
    %
    %       % Assign a sequence of waypoints
    %       waypoints = [0 0;1 1;3 4];
    %
    %       % Compute control inputs for initial pose [x y theta]
    %       [v, w] = stepInternal(pp, [0 0 0], waypoints);
    %
    %   See also robotics.PurePursuit, robotics.slalgs.internal.PurePursuit.
    
    %   Copyright 2014-2016 The MathWorks, Inc.
    %
    %   References:
    %
    %   [1] J. M. Snider, "Automatic Steering Methods for Autonomous
    %       Automobile Path Tracking", Robotics Institute, Carnegie Mellon
    %       University, Pittsburgh, PA, USA, Tech. Report CMU-RI-TR-09-08,
    %       Feb. 2009.
    %#codegen
    %#ok<*EMCA>
    
    properties
        %MaxAngularVelocity Maximum angular velocity (rad/s)
        %   The controller saturates the absolute angular velocity output
        %   at MaxAngularVelocity value. This property is tunable.
        %
        %   Default: 1.0
        MaxAngularVelocity      = 1.0
        
        %LookaheadDistance Lookahead distance (m)
        %   The lookahead distance changes the response of the controller.
        %   Higher lookahead distance produces smooth paths but the robot
        %   takes larger turns at corners. Smaller lookahead distance
        %   closely follows the path and robot takes sharp turns but may
        %   produce oscillations in the path. This property is tunable.
        %
        %   Default: 1.0
        LookaheadDistance       = 1.0
        
        %DesiredLinearVelocity Desired linear velocity (m/s)
        %   The controller assumes that the robot drives at a constant
        %   linear velocity and the computed angular velocity is
        %   independent of the linear velocity. This property is tunable.
        %
        %   Default: 0.1
        DesiredLinearVelocity   = 0.1
    end
    
    properties (Access = public)
        %ProjectionPoint Point on path closest to the current pose
        ProjectionPoint
        
        %ProjectionLineIndex Line index on which Projection point lies.
        %   Projection point is on the line index between waypoint
        %   ProjectionLineIndex and ProjectionLineIndex+1
        ProjectionLineIndex
        
        %LookaheadPoint Coordinates of the lookahead point
        LookaheadPoint
        
        %LastPose Last pose used to compute robot velocity commands
        LastPose
    end
    
    methods
        function set.MaxAngularVelocity(obj, omega)
            %set.MaxAngularVelocity Setter for Angular Velocity property
            
            validateattributes(omega, {'double', 'single'},{'nonnan', 'real', 'scalar', ...
                'positive', 'nonempty'}, 'PurePursuit', 'MaxAngularVelocity');
            obj.MaxAngularVelocity = omega;
        end
        
        function set.LookaheadDistance(obj, dist)
            %set.LookaheadDistance Setter for LookaheadDistance property
            
            validateattributes(dist, {'double', 'single'},{'nonnan', 'real', 'scalar', ...
                'positive', 'nonempty', 'finite'},'PurePursuit', 'LookaheadDistance');
            obj.LookaheadDistance = dist;
        end
        
        function set.DesiredLinearVelocity(obj, val)
            %set.DesiredLinearVelocity Setter for DesiredLinearVelocity
            
            validateattributes(val, {'double', 'single'},{'nonnan', 'real', 'scalar', ...
                'positive', 'nonempty', 'finite'},'PurePursuit', 'DesiredLinearVelocity');
            obj.DesiredLinearVelocity = val;
        end
    end
    
    methods (Access = public)
        function [v, w, lookaheadPoint, targetDir] = stepInternal(obj, currentPose, wayptsIn)
            %stepInternal Compute control commands
            
            waypts = cast(wayptsIn, 'like', currentPose);
            
            % Handle nan values
            b = ~isnan(waypts);
            nanidx = b(:,1) & b(:,2);
            waypoints = waypts(nanidx, :);
            
            if isempty(waypoints)
                v = cast(0, 'like', currentPose);
                w = cast(0, 'like', currentPose);
                targetDir = cast(0, 'like', currentPose);
                lookaheadPoint = currentPose(1:2);
                return;
            end
            
            % Find the nearest point as the projection
            computeProjectionPoint(obj, currentPose, waypoints);
            
            % Compute carrot point based on projection. If near end, then
            % the end point is the carrot point
            obj.LookaheadPoint = obj.getLookaheadPoint(waypoints);
            
            % Angle between robot heading and the line connecting robot and
            % the carrot point
            slope = atan2((obj.LookaheadPoint(2) - currentPose(2)), ...
                (obj.LookaheadPoint(1) - currentPose(1)));
            alpha = aim_angdiff(currentPose(3), slope);
            
            % Angular velocity command for a differential drive robot is
            % equal to the desired curvature to be followed by the robot.
            
            % Using eq. (2) on page 11 of Reference [1].
            w = (2*sin(alpha))/obj.LookaheadDistance;
            
            % Pick a constant rotation when robot is facing in the opposite
            % direction of the path
            if abs(abs(alpha) - cast(pi, 'like', currentPose)) < sqrt(eps(class(currentPose)))
                w = sign(w)*1;
            end
            
            w = obj.saturate(w);
            
            v = cast(obj.DesiredLinearVelocity, 'like', currentPose);
            
            lookaheadPoint = obj.LookaheadPoint;
            %Update the last pose
            obj.LastPose = [currentPose(1) currentPose(2) currentPose(3)];
            
            targetDir = alpha;
            if isnan(targetDir)
                targetDir = cast(0, 'like', currentPose);
            end
        end
    end
    
    methods (Access = protected)
        function dat = infoImpl(obj)
            %info Additional information about object status
            %   S = info(PP) returns a structure S with additional
            %   information about the current status of the PurePursuit
            %   object PP. The structure S contains the fields RobotPose,
            %   and LookaheadPoint. The RobotPose is the
            %   robot's pose that was the input to the last step function
            %   call. The LookaheadPoint is a point on the path that was
            %   used to compute the outputs of the last step function call.
            %
            %   Example:
            %       % Create a PurePursuit object
            %       pp = robotics.PurePursuit;
            %
            %       % Assign waypoints
            %       pp.Waypoints = [0 0;1 1];
            %
            %       % Compute control commands
            %       [v, w] = step(pp, [0 0 0]);
            %
            %       % Get additional information
            %       s = info(pp)
            %
            %   See also robotics.PurePursuit
            
            dat.RobotPose = obj.LastPose;
            dat.LookaheadPoint = obj.LookaheadPoint;
        end
    end
    
    methods (Access = protected)
        function avel = saturate(obj, avel)
            %saturate Saturate angular velocity
            
            if abs(avel) > obj.MaxAngularVelocity
                avel = sign(avel)*obj.MaxAngularVelocity;
            end
        end
        
        function computeProjectionPoint(obj, pose, waypoints)
            %computeProjectionPoint Find closest point past the last Projection point
            
            pose = reshape(pose,1,3);
            searchFlag = false;
            % If Projection point is not initialized, start searching from
            % first waypoint
            if obj.ProjectionLineIndex == 0
                searchFlag = true;
                obj.ProjectionPoint = waypoints(1,1:2);
                obj.ProjectionLineIndex = cast(1, 'like', pose);
            end
            
            if size(waypoints, 1) == 1
                obj.ProjectionPoint = waypoints(1,1:2);
                return;
            end
            % Start searching from the current projection line segment
            [obj.ProjectionPoint, minDistance] = ...
                closestPointOnLine(obj.ProjectionPoint, ...
                waypoints(obj.ProjectionLineIndex+1,1:2), pose(1:2));
            
            dist = norm(obj.ProjectionPoint - waypoints(obj.ProjectionLineIndex+1,1:2));
            for i = obj.ProjectionLineIndex+1:size(waypoints,1)-1
                
                if ~searchFlag && dist > obj.LookaheadDistance
                    break;
                end
                dist = dist + norm(waypoints(i,1:2) - waypoints(i+1,1:2));
                % Check the remaining waypoints
                [tempPoint, tempDistance] = ...
                    closestPointOnLine(waypoints(i,1:2), ...
                    waypoints(i+1,1:2), pose(1:2));
                
                if tempDistance < minDistance
                    minDistance = tempDistance;
                    obj.ProjectionPoint = tempPoint;
                    obj.ProjectionLineIndex = cast(i, 'like', pose);
                end
            end
        end
        
        function lookaheadPoint = getLookaheadPoint(obj, waypoints)
            %getLookaheadPoint Find the look ahead point on the path past
            %the Projection point.
            
            if size(waypoints, 1) == 1
                lookaheadPoint = waypoints(1,1:2);
                return;
            end
            
            % First check the current line segment
            dist = norm(obj.ProjectionPoint-waypoints(obj.ProjectionLineIndex+1,1:2));
            lookaheadStartPt = obj.ProjectionPoint;
            lookaheadEndPt = waypoints(obj.ProjectionLineIndex+1,1:2);
            overshootDist = dist - obj.LookaheadDistance;
            lookaheadIdx = obj.ProjectionLineIndex;
            
            % If the remaining path on current line segment is not long
            % enough for look ahead, check the waypoints past current line
            % segment.
            while overshootDist < 0 && lookaheadIdx < size(waypoints,1)-1
                lookaheadIdx = lookaheadIdx + 1;
                
                lookaheadStartPt = waypoints(lookaheadIdx,1:2);
                lookaheadEndPt = waypoints(lookaheadIdx+1,1:2);
                dist = dist + norm(lookaheadStartPt-lookaheadEndPt);
                
                overshootDist = dist - obj.LookaheadDistance;
            end
            
            % Find the exact look ahead point by interpolating between the
            % start and end point of the line segment on which the
            % lookahead point lies.
            alpha = overshootDist/norm(lookaheadStartPt - lookaheadEndPt);
            if alpha > 0
                lookaheadPoint = alpha.*lookaheadStartPt + (1-alpha).*lookaheadEndPt;
            else
                lookaheadPoint = lookaheadEndPt;
            end
        end
    end
    
    methods(Static, Access = public)
        function currentPose = validatePose(curPose, fcnName, argName)
            %validatePose Validate current pose
            validateattributes(curPose, {'double', 'single'}, ...
                {'nonempty', 'nonnan', 'finite', 'real', 'vector', 'numel', 3}, ...
                fcnName, argName);
            
            currentPose = curPose(:).';
        end
        
        function validateWaypoints(waypts, fcnName, argName)
            %validateWaypoints Validate waypoints
            validateattributes(waypts, {'double', 'single'},...
                {'nonempty', 'real','ncols',2}, ...
                fcnName, argName);
        end
    end
end
