function [F,packets,var,AverageDelayPerVehicle,AverageDelayPerPlatoon,totalVehicles,totalVehiclesCrossed] = AIM(seed,granularity,platoonMaxSize,spawnRate,duration,simSpeed,handles)
tic;
rng(seed);
packetsFromPlatoons = 0;
packetsToPlatoons = 0;
%Get Map, Show Map
intersectionData = intersection();
map = intersectionData.getMap(granularity);
show(map)
%title('Platoon-Based Intersection Management');
xlim([0 granularity]);
ylim([0 granularity]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialization and Constants
laneTraffic = zeros(1,4);
laneIsFull= zeros(1,4);
simulationTime = 0;
numVehiclesPassed = 0;
numPlatoonsPassed = 0;
currentAvgDelay = 0;
totalVehicles = [0 0 0 0];
totalVehiclesCrossed = [0 0 0 0];

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
elapsedTime = toc;
%Spawn First Platoon
laneNumber = randi([1 numOfLanes]);
platoonLeader = vehicleModel(intersectionData.spawnPoints(laneNumber,:));
%Spawn The followers
platoonSize = randi([1 platoonMaxSize]);
turnDecision = turns{randi([1 3])};
platoons(numOfPlatoons+1) = vehiclePlatoon(platoonSize,platoonLeader,turnDecision,frameCounter);
numOfPlatoons = numOfPlatoons+1;
platoons(numOfPlatoons).addVehicle(platoonSize-1);
time = 5*simSpeed;
spawnPerSec = spawnRate/3600;
lastFrame = 0;
clearTime = 0;
times = 0;
laneCapacity = 16;
fps=2;
gone=[];
%legend('Stopped Platoon','Stopped Platoon','Stopped Platoon','Stopped Platoon');
while(frameCounter<duration*fps) 
%      if(gone~=0)
%         platoons(gone) =[]; 
%         gone=0;
%     end
    
    
    set(handles.timeLabel,'String',sprintf('%.2fs',frameCounter/fps));
    set(handles.crossedVehicles,'String',sprintf('%d',sum(totalVehiclesCrossed)));
    if(mod(frameCounter,1)==0)%Spawn 4 platoons every 100 frames
    for k=1:4
        %Spawn New Platoons
        %decide lane
        %laneNumber = randi([1 numOfLanes]);
        %decide turn
        turnDecision = turns{randi([1 3])};
        %decide platoon size
        platoonSize = randi([1 platoonMaxSize]);
        %Spawn Leader
        prob = rand();
        threshold = spawnPerSec*0.5/platoonSize;
        if(prob<threshold && ((laneTraffic(k)+platoonSize)<laneCapacity))
            totalVehicles(k) = totalVehicles(k) + platoonSize;
            platoonLeader = vehicleModel(intersectionData.spawnPoints(k,:));
            platoons(numOfPlatoons+1) = vehiclePlatoon(platoonSize,platoonLeader,turnDecision,frameCounter);
            numOfPlatoons = numOfPlatoons+1;
            platoons(numOfPlatoons).addVehicle(platoonSize-1);
            laneTraffic(k) = laneTraffic(k) +platoonSize;
            if(laneTraffic(k)>laneCapacity)
                laneIsFull(k)=1;
            else 
                laneIsFull(k)=0;
            end
        end
    end
    end
    %Get Solution%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    arrivals = [];
    lengths = [];
    waitings = [];
    packetsFromPlatoons = packetsFromPlatoons + length(platoons);
    indices = 1:length(platoons);
    indices(gone) = [];
    if(isempty(indices))
        drawnow;
        frameCounter = frameCounter+1;
        F(frameCounter) = getframe(gcf);
        continue;
    else
    for j=indices
           
           if (j==1)
               waitings = [1 platoons(j).waitingTime platoons(j).arrivalTime];
               %arrivals = [1 platoons(j).arrivalTime];
           else
               waitings = [waitings; j platoons(j).waitingTime platoons(j).arrivalTime];
               %lengths = [lengths; j platoons(j).platoonSize];
           end
    end
    
    %schedule2 = sortrows(waitings,[-2]);
    %schedule = [schedule2(:,1)'];
   
    schedule2 = sortrows(waitings,[3]);
    sortedList = [schedule2(:,1)'];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    counter = 0;
    laneChecked = [0 0 0 0];
    for j=sortedList
       if(isvalid(platoons(j)))
           if(strcmp(platoons(j).state,'done'))
               gone = [gone j];
           end
           timePassed = frameCounter-lastFrame;
           if( strcmp(platoons(j).state,'stopandwait') ...    
           && timePassed>(clearTime+2) ...
                   && counter<1 && laneChecked(platoons(j).arrivalLane)==0)  
                totalVehiclesCrossed(platoons(j).arrivalLane) = totalVehiclesCrossed(platoons(j).arrivalLane) + platoons(j).platoonSize;
                packetsToPlatoons = packetsToPlatoons + 1;
                packetsFromPlatoons = packetsFromPlatoons + 1; 
                platoons(j).setPath(intersectionData.getTrajectory(platoons(j).arrivalLane,platoons(j).turn),frameCounter);
                arrival = platoons(j).arrivalTime;
                clearTime = arrival + (25+(platoons(j).platoonSize*8))/(platoons(j).linearVelocity*time); 
                lastFrame = frameCounter;
                laneTraffic(platoons(j).arrivalLane) = laneTraffic(platoons(j).arrivalLane) -platoons(j).platoonSize;
                if(laneTraffic(platoons(j).arrivalLane)>laneCapacity)
                    laneIsFull(platoons(j).arrivalLane)=1;
                else 
                    laneIsFull(platoons(j).arrivalLane)=0;
                end
                %counter = counter + 1;  
                
                for ii= sortedList(find(sortedList==j,1)+1:end)
                    if(platoons(ii).arrivalLane == platoons(j).arrivalLane)
                        platoons(ii).updateStopPoint(platoons(j).stopPoints(platoons(j).arrivalLane,:),platoons(j).arrivalLane);
                        break;
                    end
                end
           end
           laneChecked(platoons(j).arrivalLane)=1;
           drive(platoons(j),time,frameCounter,true);
           counter = counter + 1;   
           if(strcmp(platoons(j).state,'stopandwait') || strcmp(platoons(j).state,'moveandwait'))
               for ii= sortedList(find(sortedList==j,1)+1:end)
                   platoons(ii).updateStopPoint(platoons(j).tail(),platoons(j).arrivalLane);
               end
           end
           
       end
       
    end 
    drawnow;
    frameCounter = frameCounter+1;
    %platoonSizeSetting = platoonMaxSize
    %SimulationTime = frameCounter/2
    F(frameCounter) = getframe(gcf);
    %tic;
    %pause(time/1000);
    %times = [times toc-elapsedTime];
    %elapsedTime = toc;
    %platoons = sort(platoons);
end
end
aveTime = mean(times);
delays = [];
platoonDelays = [];

for j=1:length(platoons)
    if(isvalid(platoons(j)))
        if(strcmp(platoons(j).state,'done') || strcmp(platoons(j).state,'crossing'))
        delay =platoons(j).totalWaiting/fps;
            if(delay>=0)
                platoonDelays = [platoonDelays delay];
                for(i=1:platoons(j).platoonSize)
                    delays = [delays delay];
                end
            end
        elseif(platoons(j).waitingTime>=0)
            delay = platoons(j).waitingTime/fps;
                platoonDelays = [platoonDelays delay];
                for(i=1:platoons(j).platoonSize)
                    delays = [delays delay];
                end
        end
        delete(platoons(j));
    end
end
clearvars platoons
delays = delays;
platoonDelays = platoonDelays;
AverageDelayPerVehicle = mean(delays);
AverageDelayPerPlatoon = mean(platoonDelays);
packets = packetsFromPlatoons+packetsToPlatoons;
var = sum(delays.^2)/(length(delays)-1) - (length(delays))*mean(delays)^2/(length(delays)-1);
    function sol = getSchedule(lengths,waitingTimes,mode)
      [sorted sol] = sort(waitingTimes,mode);
    end
end