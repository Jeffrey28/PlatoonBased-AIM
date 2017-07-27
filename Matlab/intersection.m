classdef intersection < handle
    
    properties(SetAccess = private)
        granularity;
        lineWidth;
        lanePerRoad;
    end
    
        properties(SetAccess = public)

        spawnPoints = [20 195 0;%for Lane 1
                        205 20 pi/2;%for Lane 2
                        380 205 pi;%for Lane 3
                        195 380 -pi/2;%for Lane 4
                        ];
        stopPoints = [180 195;%for Lane 1
                        205 180;%for Lane 2
                        220 205;%for Lane 3
                        195 215;%for Lane 4
                        ];
        endPoints =  [20 205;
                        195 20;
                        380 195;
                        205 380;];
                    
        lane1To4 = [180 195;%Lane 1 Turn left
    190    195;
    195    200;
    200   205;
    202   210;
    205   220;
    205 380];
lane1To3 = [180 195;%Lane 1 Go Straight
    190    195;
    195    195;
    200   195;
    205   195;
    240   195;
    380 195];
lane1To2 = [180 195;%Lane 1 Turn right
    185    195;
    190   195;
    192    195;
    194   185;
    195   145;
    195 20];
lane2To3 = [205 180;%Lane 2 turn right
    205 185;
    205 190;
    205 192;
    215 195;
    275 195;
    380 195];
lane2To4 = [205 180;%Lane 2 go straight
    205 190;
    205 195;
    205 200;
    205 205;
    205 245;
    205 380];
lane2To1 = [205 180;
    205    190;
    205    195;
    200   200;
    190   205;
    130   205;
    20 205];%Lane 2 turn left
lane3To4 = [220 205;
    210 205;
    207 205;
    205 210;
    205 220;
    205 280;
    205 380];%Lane 3 turn right
lane3To1 = [220 205;
    210 205;
    200 205;
    160 205;
    100 205;
    50 205;
    20 205];%Lane 3 go straight
lane3To2 = [220 205;
    205 205;
    200 200;
    195 195;
    195 190;
    195 130;
    195 20];%Lane 3 turn left
lane4To1 = [195 220;
    195 210;
    190 208;
    180 205;
    130 205;
    80 205;
    20 205];%Lane 4 turn right
lane4To2 = [195 220;
    195 210;
    195 205;
    195 200;
    195 195;
    195 155;
    195 20;]%Lane 4 go straight
lane4To3 = [195 220;
    195 205;
    200 200;
    205 195;
    220 195;
    280 195;
    380 195;]%Lane 4 turn left
trajectories;             
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
methods(Access=public)
function path = getTrajectory(obj,arrivalLane,turn)
              switch turn
                  case 'right'
                      offset=1;
                  case 'straight'
                      offset=2;
                  case 'left'
                      offset=3;
              end
              index = (arrivalLane-1)*3+offset;
              path = obj.trajectories(:,:,index);
              
end
          
end
      methods(Static)
          function obj = intersection()
              obj.granularity = 400;
              obj.lineWidth = 1;
              obj.lanePerRoad = 1;
              obj.trajectories        = obj.lane1To2;%Lane 1 turn right
              obj.trajectories(:,:,2) = obj.lane1To3;%Lane 1 go straight
              obj.trajectories(:,:,3) = obj.lane1To4;%Lane 1 turn left
              obj.trajectories(:,:,4) = obj.lane2To3;%Lane 2 turn right
              obj.trajectories(:,:,5) = obj.lane2To4;%Lane 2 go straight
              obj.trajectories(:,:,6) = obj.lane2To1;%Lane 2 turn left
              obj.trajectories(:,:,7) = obj.lane3To4;%Lane 3 turn right
              obj.trajectories(:,:,8) = obj.lane3To1;%Lane 3 go straight
              obj.trajectories(:,:,9) = obj.lane3To2;%Lane 3 turn left
              obj.trajectories(:,:,10) = obj.lane4To1;%Lane 4 turn right
              obj.trajectories(:,:,11) = obj.lane4To2;%Lane 4 go straight
              obj.trajectories(:,:,12) = obj.lane4To3;%Lane 4 turn left

          end
          
          function map = getMap(varargin)
            %UNTITLED Summary of this function goes here
            %   Detailed explanation goes here
            numvarargs = length(varargin);
            if numvarargs > 3
                error('gridMapGenerator: TooManyInputs', ...
                'This function takes 1-3 input arguments');
            end
            optargs = {200 1 1};
            optargs(1:numvarargs) = varargin;
            [granularity lineWidth lanePerRoad] = optargs{:};
            obj.granularity = granularity;
            obj.lineWidth = lineWidth;
            obj.lanePerRoad = lanePerRoad;
            %granularity = 200;
            %lineWidth = 1;
            intersectionGridMap = zeros(granularity, granularity);
            bottomOfLane = 0.5*granularity-10;
            topOfLane = 0.5* granularity+10;
            intersectionGridMap(bottomOfLane:bottomOfLane+lineWidth,1:bottomOfLane)=1;
            intersectionGridMap(bottomOfLane:bottomOfLane+lineWidth,topOfLane:end)=1;
            intersectionGridMap(topOfLane:topOfLane+lineWidth,1:bottomOfLane)=1;
            intersectionGridMap(topOfLane:topOfLane+lineWidth,topOfLane:end)=1;
            intersectionGridMap(1:bottomOfLane+lineWidth,bottomOfLane:bottomOfLane+lineWidth)=1;
            intersectionGridMap(topOfLane:end,bottomOfLane:bottomOfLane+lineWidth)=1;
            intersectionGridMap(1:bottomOfLane,topOfLane:topOfLane+lineWidth)=1;
            intersectionGridMap(topOfLane:end,topOfLane:topOfLane+lineWidth)=1;
            intersectionGridMap(0.5*granularity,1:bottomOfLane)=0.5;
            intersectionGridMap(0.5*granularity,topOfLane+lineWidth:end)=0.5;
            intersectionGridMap(1:bottomOfLane,0.5*granularity)=0.5;
            intersectionGridMap(topOfLane+lineWidth:end,0.5*granularity)=0.5;
            intersectionGridMap(1:bottomOfLane,1:bottomOfLane)=0.7;
            intersectionGridMap(1:bottomOfLane,topOfLane+lineWidth:end)=0.7;
            intersectionGridMap(topOfLane+lineWidth:end,1:bottomOfLane)=0.7;
            intersectionGridMap(topOfLane+lineWidth:end,topOfLane+lineWidth:end)=0.7;
            intersectionGridMap = intersectionGridMap';
            map = OccupancyGrid(intersectionGridMap);

            end
      end
end

