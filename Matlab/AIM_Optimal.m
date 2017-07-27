function [delays,callCounter,packets,var,AverageDelayPerVehicle,AverageDelayPerPlatoon,totalVehicles,totalVehiclesCrossed] = AIM_Optimal(bal,seed,granularity,platoonMaxSize,spawnRate,duration,simSpeed,handles)
tic;
ver=2;
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
global simulationTime;
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
%F(frameCounter) = getframe(gcf);
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
simulationTime = 5*simSpeed;
spawnPerSec = spawnRate/3600;
lastFrame = 0;
clearTime = 0;
times = 0;
laneCapacity = 16;
%legend('Stopped Platoon','Stopped Platoon','Stopped Platoon','Stopped Platoon');
newBatch = 1;
fps=2;
index=0;
gone = [];
numOfPlatoonsInSchedule=0;
callCounter = 0;
    candidates = [0 0 0 0];
while(frameCounter<duration*fps) 
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
            %Spawn The followers
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
    
    schedule2 = sortrows(waitings,[3]);
    sortedList = [schedule2(:,1)'];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    counter = 0;
    timePassed = frameCounter-lastFrame;  
    lastCandidates = candidates;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(newBatch==1 || ver==2)%index>numOfPlatoonsInSchedule || index==0)
    candidates = [0 0 0 0];
    candidateCounter = 0;
    
        for pt=sortedList
            if(candidateCounter==4)
                break
            else
            lane = platoons(pt).arrivalLane;

            if(candidates(lane)==0  && ~strcmp(platoons(pt).state,'crossing') ...
              && ~strcmp(platoons(pt).state,'done')...
              && platoons(pt).arrivalTime<=clearTime)
                candidates(lane)=pt;
                candidateCounter = candidateCounter+1;
            end
            end
        end
        candidates = candidates(candidates~=0);
        if(~isempty(candidates))
            if(length(candidates)~=length(lastCandidates))
                newSchedule = greedySort(candidates);
                callCounter = callCounter+1;
                newBatch = 0;
                numOfPlatoonsInSchedule = length(newSchedule);
                index = 1;
            elseif(candidates~=lastCandidates)
                newSchedule = greedySort(candidates);
                callCounter = callCounter+1;
                newBatch = 0;
                numOfPlatoonsInSchedule = length(newSchedule);
                index = 1;
            else 
                newSchedule = greedySort(candidates);
                newBatch = 0;
                numOfPlatoonsInSchedule = length(newSchedule);
                index = 1;
            end
            
        else
            newSchedule = 0;
            index = 0;
        end
        
    end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    for j=sortedList
        if(index>0 && index <=numOfPlatoonsInSchedule)
            whosTurn = newSchedule(index);
        else
            whosTurn=0;
        end
       if(isvalid(platoons(j)))
           if(strcmp(platoons(j).state,'done'))
               gone = [gone j];
           end
           timePassed = frameCounter-lastFrame;          
           if(timePassed>=clearTime && j==whosTurn ...
               && (strcmp(platoons(j).state,'stopandwait')...
               || strcmp(platoons(j).state,'moveandwait')))
                index = index + 1;
                if(index>numOfPlatoonsInSchedule)
                    newBatch=1;
                end
                packetsToPlatoons = packetsToPlatoons + 1;
                packetsFromPlatoons = packetsFromPlatoons + 1;
                platoons(j).setPath(intersectionData.getTrajectory(platoons(j).arrivalLane,platoons(j).turn),frameCounter);
                arrival = platoons(j).arrivalTime;
                totalVehiclesCrossed(platoons(j).arrivalLane) = totalVehiclesCrossed(platoons(j).arrivalLane) + platoons(j).platoonSize;                %counter = counter + 1;
                clearTime = arrival + (25+(platoons(j).platoonSize*8))/(platoons(j).linearVelocity*simulationTime);
                lastFrame = frameCounter;
                laneTraffic(platoons(j).arrivalLane) = laneTraffic(platoons(j).arrivalLane) -platoons(j).platoonSize;
                if(laneTraffic(platoons(j).arrivalLane)>laneCapacity)
                    laneIsFull(platoons(j).arrivalLane)=1;
                else 
                    laneIsFull(platoons(j).arrivalLane)=0;
                end
                %counter = counter + 1;  
                
                for ii= sortedList(find(sortedList==j,1)+1:end)
                   platoons(ii).updateStopPoint(platoons(j).stopPoints(platoons(j).arrivalLane,:),platoons(j).arrivalLane);
                end
           end
           drive(platoons(j),simulationTime,frameCounter,true);
           if(strcmp(platoons(j).state,'stopandwait')...
                   || strcmp(platoons(j).state,'moveandwait'))
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
    %F(frameCounter) = getframe(gcf);
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
        %totalVehiclesCrossed(platoons(j).arrivalLane) = totalVehiclesCrossed(platoons(j).arrivalLane) + platoons(j).platoonSize;
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
delays = delays;
platoonDelays = platoonDelays;
AverageDelayPerVehicle = mean(delays);
AverageDelayPerPlatoon = mean(platoonDelays);
packets = packetsFromPlatoons+packetsToPlatoons;
var = sum(delays.^2)/(length(delays)-1) - (length(delays))*mean(delays)^2/(length(delays)-1);
    function sol = getSchedule(lengths,waitingTimes)
      [sorted sol] = sort(waitingTimes,'descend');
    end

function sol = getSchedule2(lengths,arrivalTimes)
      [sorted sol] = sort(arrivalTimes);
    end

    function [newSchedule] = greedySort(candidates)
        minimumDelay = inf;
        newSchedule = candidates;
        clearTimes = getClearTimes(candidates);
        permutations = perms(candidates);
        for i=1:size(permutations,1)
            maxDelay = 0;
            extraWaitTime = max(clearTime - platoons(permutations(i,1)).arrivalTime ,0);
            extraWaitTime = 0;
            if(bal==1)
            delayPerVehicle = (platoons(permutations(i,1)).waitingTime +...
                extraWaitTime)*...
                   platoons(permutations(i,1)).platoonSize;
            else
                delayPerVehicle = (platoons(permutations(i,1)).waitingTime +...
                extraWaitTime)*...
                   platoons(permutations(i,1)).platoonSize;
            end
            totalDelay = clearTimes(find(candidates ==permutations(i,1),1))+clearTime+...
                +max(platoons(permutations(i,1)).arrivalTime-clearTime,0);
             %   platoons(permutations(i,1)).waitingTime*...
              %  platoons(permutations(i,1)).platoonSize;
            %delayPerVehicle=0;
            %delayPerVehicle = delayPerVehicle +...
             %   platoons(permutations(i,1)).waitingTime*...
              %  platoons(permutations(i,1)).platoonSize;
              if(delayPerVehicle>maxDelay)
                  maxDelay = delayPerVehicle;
              end
            for j=2:size(permutations,2)
                extraWaitTime = max(totalDelay - platoons(permutations(i,j)).arrivalTime ,0);
            extraWait = extraWaitTime>0;
            if(bal==2)
                 delayPerVehicle =delayPerVehicle+j*(platoons(permutations(i,j)).waitingTime+...
                     extraWaitTime)*...
                   platoons(permutations(i,j)).platoonSize;
            else
                delayPerVehicle =delayPerVehicle+(platoons(permutations(i,j)).waitingTime+...
                     extraWaitTime)*...
                   platoons(permutations(i,j)).platoonSize;
            end
               %platoons(permutations(i,j)).waitingTime+
               %laneTraffic(platoons(permutations(i,j)).arrivalLane);

               %platoons(permutations(i,j)).platoonSize; 
               % + ...
                totalDelay = totalDelay+max(platoons(permutations(i,j)).arrivalTime - totalDelay,0) + ...
                    clearTimes(find(candidates ==permutations(i,j),1));
                 if(delayPerVehicle>maxDelay)
                    maxDelay = delayPerVehicle;
                 end
            end
            if(delayPerVehicle<minimumDelay)
                minimumDelay = delayPerVehicle;
                newSchedule = permutations(i,:);
            end
        end
        
    end
    function clearTimes = getClearTimes(candidates)
        for i=1:length(candidates)
            if(candidates(i)==0)
                pSize(i) = 0;
            else
                pSize(i) = platoons(candidates(i)).platoonSize;
            end
        end
        constants = zeros(1,length(candidates)).*25;
        clearTimes = (constants+(pSize.*8))./(simulationTime); 
        
    end
end