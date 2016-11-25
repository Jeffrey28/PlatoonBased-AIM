function [F] = AIM(granularity,platoonMaxSize,spawnRate,duration)
tic;
%Get Map, Show Map
intersectionData = intersection();
map = intersectionData.getMap(granularity);
show(map)
title('Platoon-Based Intersection Management');
xlim([0 granularity]);
ylim([0 granularity]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialization and Constants
laneTraffic = zeros(1,4);
simulationTime = 0;
numVehiclesPassed = 0;
numPlatoonsPassed = 0;
currentAvgDelay = 0;

%LaneTail is initialized to the stop points, this will dynamically change..
%as platoon stop for their turn.other platoons will use this data to stop
%right behind the last platoon in queue.
laneStop = [180 195;%for Lane 1
                        205 180;%for Lane 2
                        220 205;%for Lane 3
                        195 215;%for Lane 4
                        ];



turns ={'straight' 'left' 'right'};
numOfPlatoons = 0;
numOfLanes = 4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
frameCounter = 1;
F(frameCounter) = getframe(gcf);
%legend('Vehicle');
%stoppedByUser=0;
tic;
elapsedTime = toc
%Spawn First Platoon
platoonLeader = vehicleModel(intersectionData.spawnPoints(1,:));
%Spawn The followers
platoons(numOfPlatoons+1) = vehiclePlatoon(2,platoonLeader,'right');
numOfPlatoons = numOfPlatoons+1;
platoons(numOfPlatoons).addVehicle(1);
time = 1.5;
while(elapsedTime<duration) 
    if(mod(frameCounter,50)==0)%Spawn 4 platoons every 100 frames
    for k=1:4
    %Spawn New Platoons
    %decide lane
    laneNumber =k;% randi([1 numOfLanes]);
    %decide turn
    turnDecision = turns{randi([1 3])};
    %decide platoon size
    platoonSize = randi([1 platoonMaxSize]);
    %Spawn Leader
    platoonLeader = vehicleModel(intersectionData.spawnPoints(laneNumber,:));
    %Spawn The followers
    platoons(numOfPlatoons+1) = vehiclePlatoon(platoonSize,platoonLeader,turnDecision);
    numOfPlatoons = numOfPlatoons+1;
    platoons(numOfPlatoons).addVehicle(platoonSize-1);
    end
    end
    for j=1:length(platoons)
       if(isvalid(platoons(j)))
           %check if pass granted...
           %assign trajectory
           rnd = randi([1 5]);
           if(platoons(j).arrivalLane==1 || platoons(j).arrivalLane==2)
           if(strcmp(platoons(j).state,'moveandwait') && rnd ==5)      
                platoons(j).setPath(intersectionData.getTrajectory(platoons(j).arrivalLane,platoons(j).turn));
           end
           
           end
           %drive current platoon
            drive(platoons(j),time);
       end
    end
    drawnow;
    frameCounter = frameCounter+1;
    F(frameCounter) = getframe(gcf);
    %tic;
    pause(time/1000);
    elapsedTime = toc;
end
for j=1:length(platoons)
    if(isvalid(platoons(j)))
        delete(platoons(j));
    end
end