classdef vehicleModel < handle
      
    properties(SetAccess = public)
        %Dt Time step for simulation of the robot
        Dt = 1;
        HeadingVectorLength = 2.0;

    end
    
    properties(SetAccess = public)
        %CurrentPose Current pose of the robot
        CurrentPose;
        Type = 'follower';
        
        %Trajectory Pose history of the robot
        Trajectory = [];
    end
    
    properties(Access = private)
        %HeadingVectorLength Length of the heading vector for plotting        
        %HRobot Graphics handle for robot
        HRobot;
        
        %HRobotHeading Graphics handle for robot heading
        HRobotHeading;
        
        %HRobotTrail Graphics handle for robot pose history
        HRobotTrail;
    end
    
    methods
        function obj = vehicleModel(currentPose)            
            obj.CurrentPose = currentPose;
            obj.Type = 'leader';
            robotCurrentLocation = obj.CurrentPose(1:2);
            
            obj.Trajectory = obj.CurrentPose;
            ax = gca;
            hold(ax, 'on');
            [x, y] = obj.getVehicleBody(obj.CurrentPose, obj.HeadingVectorLength);
            % Draw robot
            obj.HRobot = plot(x,y, 'm', 'Parent', ax);
            %obj.HRobot = plot(ax, x, y, 'g', 'LineWidth', 3);
            
            % Draw heading vector
            [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
            obj.HRobotHeading = plot(ax, xVec, yVec, 'k', 'LineWidth', 3);
            
            % Draw robot trail
            obj.HRobotTrail = plot(ax, robotCurrentLocation(1),  robotCurrentLocation(2), 'LineWidth', 2);
            
            hold(ax, 'off');
        end
        function setRadius(obj,robotRadius)
            obj.HeadingVectorLength = robotRadius;
        end
        function drive(obj, v, w,dt,draw)
            %drive Drive the robot by integrating the kinematics
            %   drive(OBJ, V, W) updates the current pose of the robot by
            %   integrating the equations of the motion with linear
            %   velocity input V and angular velocity input W.
            curPose = obj.CurrentPose;
            x = curPose(1) + v*cos(curPose(3))*dt;
            y = curPose(2) + v*sin(curPose(3))*dt;
            theta = curPose(3) + w*dt;
            obj.CurrentPose = [x, y, theta];
            obj.Trajectory = [obj.Trajectory; obj.CurrentPose];
            if draw
                updatePlots(obj);                
            end
        end
    end
    
    methods(Access = private)
        function updatePlots(obj)
            %updatePlots Update all plots
            
            updateHeading(obj);
            updateRobotCurrentLocation(obj);
            %updateRobotTrail(obj);
            %drawnow;
        end
        function updateHeading(obj)
            %updateRobotCurrentLocation Update the X/YData for heading vector
            [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
            obj.HRobotHeading.XData = xVec;
            obj.HRobotHeading.YData = yVec;
        end
        function updateRobotCurrentLocation(obj)
            %updateRobotCurrentLocation Update the X/YData for robot plot
            [robotXData, robotYData] = obj.getVehicleBody(obj.CurrentPose, obj.HeadingVectorLength);
            obj.HRobot.XData =  robotXData;
            obj.HRobot.YData =  robotYData;
        end
        function updateRobotTrail(obj)
            %updateRobotTrail Update the X/YData for pose history
            robotCurrentLocation = obj.CurrentPose(1:2);
            obj.HRobotTrail.XData = [obj.HRobotTrail.XData  robotCurrentLocation(1)];
            obj.HRobotTrail.YData = [obj.HRobotTrail.YData  robotCurrentLocation(2)];
        end
    end
    
    methods(Access = private, Static)
        function [xVec, yVec] = computeHeadingVector(robotCurrentPose, robotDiameter)
            %computeHeadingVector Compute XData and YData for heading vector
            
            robotCurrentLocation = robotCurrentPose(1:2);
            cth = cos(robotCurrentPose(3));
            sth = sin(robotCurrentPose(3));
            
            dx = robotDiameter * cth;
            dy = robotDiameter * sth;
            
            xVec = [ robotCurrentLocation(1)   robotCurrentLocation(1)+dx];
            yVec = [ robotCurrentLocation(2)   robotCurrentLocation(2)+dy];
        end
        
        function [x, y] = getCircleCoordinate(pose, r)
            %getCircleCoordinate Compute XData and YData for a circle
            %   This function computes the XData and YData for updating the
            %   graphics object.
            theta = linspace(0,2*pi,20);
            x = r*cos(theta) + pose(1);
            y = r*sin(theta) + pose(2);
        end
        
        function [xVec, yVec] = getVehicleBody(robotCurrentPose, robotDiameter)
            angle = robotCurrentPose(3);
            vehicle_body = [-robotDiameter*2 robotDiameter*2 -robotDiameter*2 ...
                robotDiameter*2 -robotDiameter*2 -robotDiameter*2 ...
                robotDiameter*2 robotDiameter*2; ...
                robotDiameter robotDiameter -robotDiameter -robotDiameter ...
                robotDiameter -robotDiameter robotDiameter -robotDiameter];
            R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
            vehicle_body = R * vehicle_body;
            xVec = vehicle_body(1,:) + robotCurrentPose(1);
            yVec = vehicle_body(2,:) + robotCurrentPose(2);
            
%             robotCurrentLocation = robotCurrentPose(1:2);
%             cth = cos(robotCurrentPose(3));
%             sth = sin(robotCurrentPose(3));
%             
%             dx = robotDiameter * cth;
%             dy = robotDiameter * sth;
%             
%             xVec = [ robotCurrentLocation(1)-dx   robotCurrentLocation(1)+dx];
%             yVec = [ robotCurrentLocation(2)-dy   robotCurrentLocation(2)+dy];
        end
        
    end
end

