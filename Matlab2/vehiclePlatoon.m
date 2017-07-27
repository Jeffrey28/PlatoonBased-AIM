classdef vehiclePlatoon < handle
      
    properties(SetAccess = public)
        totalWaiting = -1;
        expectedArrivalTime = 0;
        waitingTime = 0;
        delay = 0;
        distt = 123;
        platoonSize;
        numPlatoonPassengers;
        platoonCapacity;
        hasLeader = 0;
        vehicles ;
        isUpdated=0;
        controllers;
        leader;
        linearVelocity = 1;
        angularVelocity = 1;
        lookaheadDistance =10;
        desiredDistance = 10;
        path;
        state ;
        arrivalLane = 1;
        platoonVelocity = 1;
        arrivalTime=Inf;
        timeStep = 5;
        color = 'b';
        turn = 'straight';%left %right
        
        spawnPoints = [20 195;%for Lane 1
                        205 20;%for Lane 2
                        380 205;%for Lane 3
                        195 380;%for Lane 4
                        ];
        stopPoints = [180 195;%for Lane 1
                        205 180;%for Lane 2
                        220 205;%for Lane 3
                        195 215;%for Lane 4
                        ];
                        
    end
    
    methods (Access = private)
        function controller = getController(obj)
            controller =  PurePursuit;
            controller.Waypoints = obj.path;
            controller.DesiredLinearVelocity = obj.linearVelocity;
            controller.MaxAngularVelocity = obj.angularVelocity;
            controller.LookaheadDistance = obj.lookaheadDistance;
        end
        
        function res = isArrived(obj)
            res = 0;
            desiredDistanceToStop = 2.5;
            distanceToStopPoint = getDistanceToStop2(obj);
            if distanceToStopPoint < desiredDistanceToStop 
                  res = 1;%The leader is right behind the stop point, the platoon should stop
            end 
        end
    end
    methods
        function delete(obj)
            for i=1:obj.platoonSize
                delete(obj.vehicles(i));
            end
        end
        function waiting = getSumOfWaitings(obj)
            waiting = obj.platoonSize*obj.waitingTime;
        end
        function obj = vehiclePlatoon(capacity,leader,turn,frameCounter)
            if(nargin==0)
                obj.turn = 0;
            else
            obj.turn = turn;
            obj.platoonCapacity = capacity;
            obj.controllers = cell(1,obj.platoonCapacity);
            obj.vehicles = leader;
            obj.platoonSize = 1;
            obj.leader = obj.vehicles(1);
            obj.state = 'moveandwait';
            obj.color = 'b';
            %obj.arrivalLane=1;
            if(leader.CurrentPose(1:2) == obj.spawnPoints(1,:))
                obj.arrivalLane = 1;
            elseif (leader.CurrentPose(1:2) == obj.spawnPoints(2,:))
                obj.arrivalLane = 2;
                elseif (leader.CurrentPose(1:2) == obj.spawnPoints(3,:))
                    obj.arrivalLane = 3;
                    elseif (leader.CurrentPose(1:2) == obj.spawnPoints(4,:))
                        obj.arrivalLane = 4;
            end
            obj.path = obj.stopPoints(obj.arrivalLane,:);
            obj.arrivalTime = getArrivalTime(obj,frameCounter);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function arrivalTime = getArrivalTime(obj,frameCounter)
            arrivalTime = Inf;
            if(strcmp(obj.state,'moveandwait'))
                dist = getDistanceToStop(obj);
                obj.distt = dist;
                temp = (obj.linearVelocity*obj.timeStep);
                if(temp ==0)
                    vel = obj.platoonVelocity
                    tstep = obj.timeStep
                end
                arrivalTime = dist/temp;
                if(obj.expectedArrivalTime==0 && arrivalTime~=Inf )
                    obj.expectedArrivalTime = arrivalTime+frameCounter;
                end
            end
            if(strcmp(obj.state,'stopandwait'))  
                dist = getDistanceToStop(obj);
                arrivalTime = dist/(obj.linearVelocity*obj.timeStep);
                if(obj.expectedArrivalTime==0 && arrivalTime~=Inf )
                    obj.expectedArrivalTime = arrivalTime+frameCounter;
                end
            end
            arrivalTime = max(arrivalTime,0);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function dist = getDistanceToStop(obj)
            switch obj.arrivalLane
                case 1
                    %disp('lane1');
                    dist = obj.stopPoints(1,1) - obj.vehicles(1).CurrentPose(1);   
                case 2
                    %disp('lane2');
                     dist = obj.stopPoints(2,2) -obj.vehicles(1).CurrentPose(2);
                case 3
                    %disp('lane3');
                    dist = obj.vehicles(1).CurrentPose(1) - obj.stopPoints(3,1);
                case 4
                    %disp('lane4');
                    dist = obj.vehicles(1).CurrentPose(2) - obj.stopPoints(4,2);
            end
        end
        function dist = getDistanceToStop2(obj)
            switch obj.arrivalLane
                case 1
                    %disp('lane1');
                    dist = obj.path(1) - obj.vehicles(1).CurrentPose(1);   
                case 2
                    %disp('lane2');
                     dist = obj.path(2) -obj.vehicles(1).CurrentPose(2);
                case 3
                    %disp('lane3');
                    dist = obj.vehicles(1).CurrentPose(1) - obj.path(1);
                case 4
                    %disp('lane4');
                    dist = obj.vehicles(1).CurrentPose(2) - obj.path(2);
            end
        end
        function setPath(obj,path,frameCounter)
            obj.path = path;
            obj.totalWaiting = obj.waitingTime;
            obj.waitingTime=-1;
            obj.controllers{1} = getController(obj);
            obj.state = 'crossing';
            obj.color = 'g';
%             dist = getDistanceToStop(obj);
%             if(dist>1)
%                 calculatedDelay = (frameCounter + dist/(obj.platoonVelocity*obj.timeStep)) - obj.expectedArrivalTime;
%             else
%                 calculatedDelay = frameCounter - obj.expectedArrivalTime;
%             end
%             obj.delay = calculatedDelay;
        end
        function updateStopPoint(obj,path,lane)
            if(lane==obj.arrivalLane && ...
                    (strcmp(obj.state,'moveandwait') ...
                || strcmp(obj.state,'stopandwait')))
                obj.path = path;
                obj.controllers{1} = getController(obj);
                obj.isUpdated = 1;
                %obj.state = 'moveandwait';
            end
        end
        function pose = head(obj)
            pose = obj.leader.CurrentPose(1:2);
            [x ,y] = pol2cart(pose(3),obj.desiredDistance);
            pose = [-x-pose(1) -y-pose(2)];
        end
        
        function tail = tail(obj)
            pose = obj.vehicles(obj.platoonSize).CurrentPose;
            [x ,y] = pol2cart(pose(3),obj.desiredDistance+2.5);
            tail = [-x+pose(1) -y+pose(2)];
        end
            
        function addVehicle(obj,num)
            if num+obj.platoonSize > obj.platoonCapacity 
                error('Number of  Vehicles is greater than the capacity');
            end
            for i=1:num
                tail = obj.vehicles(obj.platoonSize).CurrentPose;
                [x ,y] = pol2cart(tail(3),obj.desiredDistance);
                followerPosition = [-x+tail(1) -y+tail(2) tail(3)];
                newFollower = vehicleModel(followerPosition);
                newController =  PurePursuit;
                newController.Waypoints = obj.vehicles(obj.platoonSize).CurrentPose(1:2);
                newController.DesiredLinearVelocity = obj.linearVelocity;
                newController.MaxAngularVelocity = obj.angularVelocity;
                newController.LookaheadDistance = obj.lookaheadDistance;
                obj.platoonSize = obj.platoonSize+1;
                obj.controllers{1,obj.platoonSize} = newController;
                obj.vehicles(obj.platoonSize) = newFollower;
                
            end
           
        end
        function drive(obj,dt,frameCounter,draw)
            obj.timeStep = dt;
            if(obj.isUpdated==1)
                obj.state='moveandwait';
                obj.color = 'b';
                obj.isUpdated = 0;
                if isArrived(obj)
                        obj.state = 'stopandwait';
                        obj.color = 'r';
                        obj.platoonVelocity = 0;
                
                end
            end
            switch (obj.state)
                case 'crossing'
                    if(sqrt(sum((obj.leader.CurrentPose(1:2) - obj.path(6,:)) .^ 2))>10)
                    for i=1:obj.platoonSize
                        if i>1
                            obj.controllers{1,i}.Waypoints = obj.vehicles(i-1).CurrentPose(1:2);
                        end
                        [v, omega] = step(obj.controllers{1,i}, obj.vehicles(i).CurrentPose);
                        drive(obj.vehicles(i),v,omega,dt,draw,obj.color);
                        obj.platoonVelocity = v;
                    end
                    else
                        for i=1:obj.platoonSize
                            obj.vehicles(i).CurrentPose = [1000 1000 0];
                            drive(obj.vehicles(i),0,0,dt,true,obj.color);
                        end        
                        obj.state = 'done';
                        obj.color = 'k';
                        %fprintf('Arrival Lane = %d DONE!\n',obj.arrivalLane);
                    end
                    %drawnow;
                case 'moveandwait'
                    
                    if isArrived(obj)
                        obj.state = 'stopandwait';
                        obj.color = 'r';
                        obj.platoonVelocity = 0;
                        %fprintf('Arrival Lane = %d Stopped!\n',obj.arrivalLane);
                    else
                        %disp('Not Arrived Yet!');
                    for i=1:obj.platoonSize
                        if (i==1)
                            %newController = obj.controllers{1,i};
                            newController =  PurePursuit;
                            newController.DesiredLinearVelocity = obj.linearVelocity;
                            newController.MaxAngularVelocity = obj.angularVelocity;
                            newController.LookaheadDistance = obj.lookaheadDistance;
                            [x ,y] = pol2cart(obj.vehicles(i).CurrentPose(3),obj.desiredDistance);
                            distance = [x+obj.vehicles(i).CurrentPose(1) y+obj.vehicles(i).CurrentPose(2)];
                            newController.Waypoints = distance;
                            obj.controllers{1,i} = newController;
                        else
                            
                            obj.controllers{1,i}.Waypoints = obj.vehicles(i-1).CurrentPose(1:2);
                        end
                        
                        [v, omega] = step(obj.controllers{1,i}, obj.vehicles(i).CurrentPose);
                        obj.platoonVelocity = v;
                        drive(obj.vehicles(i),v,omega,dt,draw,obj.color);
                    end
                    end
                    
                case 'stopandwait'
                    obj.color = 'r';
                    obj.platoonVelocity = 0;
                    obj.waitingTime = obj.waitingTime+1;
                    for i=1:obj.platoonSize
                        drive(obj.vehicles(i),0,0,dt,true,obj.color);
                    end
                    
                case 'deleted'
                    %TODO: complete this later
                    
                case 'done'
                    %move platoon out of the simulation space.
%                     for i=1:obj.platoonSize
%                         obj.vehicles(i).CurrentPose = [1000 1000 0];
%                         drive(obj.vehicles(i),0,0,dt,true,obj.color);
%                     end        
            end
           obj.arrivalTime = getArrivalTime(obj,frameCounter);
        end
    end
end