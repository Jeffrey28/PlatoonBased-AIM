classdef vehiclePlatoon < handle
      
    properties(SetAccess = public)
        platoonSize;
        numPlatoonPassengers;
        platoonCapacity;
        hasLeader = 0;
        vehicles ;
        controllers;
        leader;
        linearVelocity = 1;
        angularVelocity = 1;
        lookaheadDistance = 10;
        desiredDistance = 10;
        path;
        state ;
        arrivalLane = 1;
        platoonVelocity = 0;
        timeStep = 0;
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
             controller = robotics.PurePursuit;
            controller.Waypoints = obj.path;
            controller.DesiredLinearVelocity = obj.linearVelocity;
            controller.MaxAngularVelocity = obj.angularVelocity;
            controller.LookaheadDistance = obj.lookaheadDistance;
        end
        
        function res = isArrived(obj)
            res = 0;
            desiredDistanceToStop = 2;
            distanceToStopPoint = getDistanceToStop(obj);
            if distanceToStopPoint < desiredDistanceToStop && distanceToStopPoint > 0
                  res = 1;%The leader is right behind the stop point, the platoon should stop
            end 
        end
    end
    methods
        function obj = vehiclePlatoon(capacity,leader,turn)
            obj.turn = turn;
            obj.path = path;
            obj.platoonCapacity = capacity;
            obj.controllers = cell(1,obj.platoonCapacity);
            obj.vehicles = leader;
            obj.platoonSize = 1;
            obj.leader = obj.vehicles(1);
            obj.state = 'moveandwait';
            obj.arrivalLane=1;
            if(leader.CurrentPose(1:2) == obj.spawnPoints(1,:))
                obj.arrivalLane = 1;
            elseif (leader.CurrentPose(1:2) == obj.spawnPoints(2,:))
                obj.arrivalLane = 2;
                elseif (leader.CurrentPose(1:2) == obj.spawnPoints(3,:))
                    obj.arrivalLane = 3;
                    elseif (leader.CurrentPose(1:2) == obj.spawnPoints(4,:))
                        obj.arrivalLane = 4;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function arrivalTime = getArrivalTime(obj)
            dist = getDistanceToStop(obj);
            if(obj.platoonVelocity ~=0)
                arrivalTime = dist/obj.platoonVelocity;
            else
                arrivalTime = inf;
            end
            
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
        function setPath(obj,path)
            obj.path = path;
            obj.controllers{1} = getController(obj);
            obj.state = 'passing';
        end
        function pose = head(obj)
            pose = obj.leader.CurrentPose(1:2);
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
                newController = robotics.PurePursuit;
                newController.Waypoints = obj.vehicles(obj.platoonSize).CurrentPose(1:2);
                newController.DesiredLinearVelocity = obj.linearVelocity;
                newController.MaxAngularVelocity = obj.angularVelocity;
                newController.LookaheadDistance = obj.lookaheadDistance;
                obj.platoonSize = obj.platoonSize+1;
                obj.controllers{1,obj.platoonSize} = newController;
                obj.vehicles(obj.platoonSize) = newFollower;
                
            end
           
        end
        function drive(obj,dt)
            obj.timeStep = dt;
            switch (obj.state)
                case 'passing'
                    if(sqrt(sum((obj.leader.CurrentPose(1:2) - obj.path(end,:)) .^ 2))>10)
                    for i=1:obj.platoonSize
                        if i>1
                            obj.controllers{1,i}.Waypoints = obj.vehicles(i-1).CurrentPose(1:2);
                        end
                        [v, omega] = step(obj.controllers{1,i}, obj.vehicles(i).CurrentPose);
                        drive(obj.vehicles(i),v,omega,dt,true);
                        obj.platoonVelocity = v;
                    end
                    else
                        for i=1:obj.platoonSize
                        drive(obj.vehicles(i),0,0,dt,true);
                        end
                        obj.state = 'done';
                        fprintf('Arrival Lane = %d DONE!\n',obj.arrivalLane);
                    end
                    %drawnow;
                case 'moveandwait'
                    
                    if isArrived(obj)
                        obj.state = 'stopandwait';
                        fprintf('Arrival Lane = %d Stopped!\n',obj.arrivalLane);
                    else
                        %disp('Not Arrived Yet!');
                    for i=1:obj.platoonSize
                        if (i==1)
                            %newController = obj.controllers{1,i};
                            newController = robotics.PurePursuit;
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
                        
                        drive(obj.vehicles(i),v,omega,dt,true);
                    end
                    end
                    %drawnow;
                case 'stopandwait'
                    %drawnow;
                case 'deleted'
                    
                case 'done'
                    for i=1:obj.platoonSize
                        obj.vehicles(i).CurrentPose = [1000 1000 0];
                        drive(obj.vehicles(i),0,0,dt,true);
                    end        
            end
           
        end
    end
end