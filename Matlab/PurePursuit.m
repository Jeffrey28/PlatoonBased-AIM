classdef PurePursuit < PurePursuitBase
    %PUREPURSUIT Create a controller to follow a set of waypoints
    %   The pure pursuit controller is a geometric controller for following
    %   a path. Given a set of waypoints, the pure pursuit controller
    %   computes linear and angular velocity control inputs for a given
    %   pose of a differential drive robot.
    %
    %   PP = robotics.PUREPURSUIT returns a pure pursuit system object, PP,
    %   that computes linear and angular velocity inputs for a differential
    %   drive robot using the PurePursuit algorithm.
    %
    %   PP = robotics.PUREPURSUIT('PropertyName', PropertyValue, ...) returns
    %   a pure pursuit object, PP, with each specified property set to
    %   the specified value.
    %
    %   Step method syntax:
    %
    %   [V, W] = step(PP, POSE) finds the linear velocity, V, and the
    %   angular velocity, W, for a 3-by-1 input vector POSE using the pure
    %   pursuit algorithm. The POSE is the current position,
    %   [x y orientation] of the robot. The output velocities V and W can
    %   be applied to a real or simulated differential drive robot to drive
    %   it along the desired waypoint sequence.
    %
    %   [V, W, LOOKAHEADPOINT] = step(PP, POSE) returns 1-by-2 array
    %   LOOKAHEADPOINT, which is an [x y] location on the path used to compute
    %   velocity commands. The location on the path is computed based on
    %   the LookaheadDistance property.
    %
    %   The Waypoints can contain NaN values. The rows containing NaN values
    %   will be ignored. If all Waypoints are NaN, then zero velocity output
    %   will be returned.
    %
    %   System objects may be called directly like a function instead of using
    %   the step method. For example, y = step(obj, x) and y = obj(x) are
    %   equivalent.
    %
    %   PUREPURSUIT methods:
    %
    %   step        - Compute linear and angular velocity control commands
    %   release     - Allow property value changes
    %   reset       - Reset internal states to default
    %   clone       - Create pure pursuit object with same property values
    %   isLocked    - Locked status (logical)
    %   info        - Get additional information about the object
    %
    %   PUREPURSUIT properties:
    %
    %   Waypoints               - Waypoints representing a path to follow
    %   MaxAngularVelocity      - Desired maximum angular velocity
    %   LookaheadDistance       - Lookahead distance to compute controls
    %   DesiredLinearVelocity   - Desired constant linear velocity
    %
    %   Example
    %
    %       % Create a pure pursuit object
    %       pp = robotics.PurePursuit;
    %
    %       % Assign a sequence of waypoints
    %       pp.Waypoints = [0 0;1 1;3 4];
    %
    %       % Compute control inputs for initial pose [x y theta]
    %       [v, w] = step(pp, [0 0 0]);
    %
    %       % Compute lookahead point
    %       [v, w, lookaheadPoint] = step(pp, [0 0 0]);
    %
    %   See also robotics.BinaryOccupancyGrid, robotics.PRM.
    
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
        %Waypoints The waypoints representing a path to follow
        %
        %   Default: []
        Waypoints
    end
    
    methods
        function set.Waypoints(obj, waypts)
            %set.Waypoints Setter for Waypoints property
            
            obj.validateWaypoints(waypts, 'PurePursuit', 'Waypoints');
            obj.Waypoints = waypts;
        end
        
        function obj = PurePursuit(varargin)
            %PurePursuit Constructor
            setProperties(obj,nargin,varargin{:},'Waypoints', ...
                'DesiredLinearVelocity', 'MaxAngularVelocity', ...
                'LookaheadDistance');
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj,curPose)
            obj.LookaheadPoint = zeros(1,2, 'like', curPose);
            obj.LastPose = zeros(1,3, 'like', curPose);
            obj.ProjectionPoint = nan(1,2, 'like', curPose);
            obj.ProjectionLineIndex = cast(0, 'like', curPose);
        end
        
        function validateInputsImpl(obj, curPose)
            %validateInputsImpl Validate inputs before setupImpl is called
            coder.internal.errorIf(isempty(obj.Waypoints), ...
                'robotics:robotalgs:purepursuit:EmptyWaypoints');
            
            obj.validatePose(curPose, 'step', 'pose');
        end
        
        function [v, w, lookaheadPoint] = stepImpl(obj,curPose)
            %stepImpl Compute control commands
            
            currentPose = obj.validatePose(curPose, 'step', 'pose');
            [v, w, lookaheadPoint] = obj.stepInternal(currentPose, obj.Waypoints);
        end
        
        function num = getNumInputsImpl(~)
            %getNumInputsImpl return number of inputs
            
            % Input is current pose
            num = 1;
        end
        
        function num = getNumOutputsImpl(~)
            %getNumOutputsImpl return number of outputs
            
            % Output is linear velocity, angular velocity and look ahead
            % point.
            num = 3;
        end
        
        
        function flag = isInputSizeLockedImpl(~,~)
            flag = true;
        end
        
        
        function processTunedPropertiesImpl(obj)
            %processTunedPropertiesImpl Perform calculations if tunable
            % properties change between calls to steps
            
            % Detect waypoints change
            waypointsChange = isChangedProperty(obj, 'Waypoints');
            if waypointsChange
                obj.ProjectionLineIndex = cast(0, 'like', obj.ProjectionLineIndex);
            end
        end
        
        function s = saveObjectImpl(obj)
            %saveObjectImpl Custom save implementation
            s = saveObjectImpl@matlab.System(obj);
            
            s.ProjectionPoint = obj.ProjectionPoint;
            s.LookaheadPoint = obj.LookaheadPoint;
            s.LastPose = obj.LastPose;
            s.ProjectionLineIndex = obj.ProjectionLineIndex;
        end
        
        function loadObjectImpl(obj, svObj, wasLocked)
            %loadObjectImpl Custom load implementation
            
            obj.LookaheadPoint = svObj.LookaheadPoint;
            obj.LastPose = svObj.LastPose;
            
            if ~isfield(svObj, 'ProjPointIdx')
                % If object is saved using current release (i.e. 16b onwards)
                
                obj.ProjectionPoint = svObj.ProjectionPoint;
                obj.ProjectionLineIndex = svObj.ProjectionLineIndex;
            elseif ~isempty(svObj.LastPose)
                % Ensure 15a-16a release objects are loaded accurately
                
                % Find index of each waypoint
                coder.varsize('tempIndex');
                tempIndex = 1;
                for i=1:size(svObj.Waypoints,1)
                    distToPt = sum((svObj.Path - svObj.Waypoints(i,:)).^2,2);
                    tempIndex = [tempIndex find(distToPt < sqrt(eps)).'];  %#ok<AGROW>
                end
                index = sort(unique(tempIndex));
                higherIdx = index > svObj.ProjPointIdx;
                obj.ProjectionLineIndex = max(find(higherIdx, 1)-1, 1);
                obj.ProjectionPoint = svObj.Path(svObj.ProjPointIdx, :);
                computeProjectionPoint(obj, svObj.LastPose, svObj.Waypoints);
            else
                % Uninitialized 15a-16a objects
                % Only double datatype was supported in 15a-16a
                obj.ProjectionPoint = nan(1,2);
                obj.LookaheadPoint = zeros(1,2);
                obj.LastPose = zeros(1,3);
                obj.ProjectionLineIndex = 0;
            end
            
            if isempty(svObj.Waypoints)
                % Prevent warning for uninitialized object
                return;
            else
                % Call base class method
                loadObjectImpl@matlab.System(obj,svObj,wasLocked);
            end
        end
        
        function resetImpl(obj)
            %resetImpl Reset the internal state to defaults
            
            % To preserve datatype, reset using multiplication
            obj.ProjectionPoint = nan*obj.ProjectionPoint;
            obj.LookaheadPoint = 0*obj.LookaheadPoint;
            obj.LastPose = 0*obj.LastPose;
            obj.ProjectionLineIndex = 0*obj.ProjectionLineIndex;
        end
    end
    
end
