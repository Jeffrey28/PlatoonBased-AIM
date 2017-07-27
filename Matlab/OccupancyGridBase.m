classdef (Hidden, Abstract) OccupancyGridBase < handle
    %This class is for internal use only. It may be removed in the future.
    
    %OccupancyGridBase Define the OccupancyGridBase class.
    % Common base class between occupancy grid objects
    
    %   Copyright 2014-2016 The MathWorks, Inc.
    
    %#codegen
    
    properties (SetAccess = protected)
        %GridSize Size of the grid
        %   A vector [ROWS COLS] indicating the size of the grid as number of
        %   rows and columns.
        GridSize = [0 0]
        
        %Resolution Grid resolution in cells per meter
        %
        %   Default: 1
        Resolution = 1
    end
    
    properties (Dependent, SetAccess = protected)
        %XWorldLimits Minimum and maximum values of X
        %   A vector [MIN MAX] representing the world limits of the grid
        %   along the X axis.
        XWorldLimits
        
        %YWorldLimits Minimum and maximum values of Y
        %   A vector [MIN MAX] representing the world limits of the grid
        %   along the Y axis.
        YWorldLimits
    end
    
    properties
        %GridLocationInWorld - Location of the grid in world coordinates
        %   A vector defining the [X, Y] world coordinates of the bottom-left
        %   corner of the grid.
        %   Default: [0 0]
        GridLocationInWorld = [0 0]
    end
    
    properties (Access = protected)
        %AxesTag Axes tag used for quick show updates
        AxesTag = 'OccupancyGrid';
        
        %ImageTag Image tag used for quick show update
        ImageTag = 'OccupancyGrid';
    end
    
    methods
        function xlims = get.XWorldLimits(obj)
            %get.XWorldLimits Getter for XWorldLimits property
            xlims = zeros(1,2);
            xlims(1,1) = obj.GridLocationInWorld(1);
            % Grid is in column-major format hence using 2nd dimension
            xlims(1,2) = obj.GridLocationInWorld(1) + ...
                obj.GridSize(1,2)/obj.Resolution;
        end
        
        function ylims = get.YWorldLimits(obj)
            %get.YWorldLimits Getter for YWorldLimits property
            ylims = zeros(1,2);
            ylims(1,1) = obj.GridLocationInWorld(2);
            % Grid is in column-major format hence using 1st dimension
            ylims(1,2) = obj.GridLocationInWorld(2) + ...
                obj.GridSize(1,1)/obj.Resolution;
        end
        
        function set.GridLocationInWorld(obj, loc)
            %set.GridLocationInWorld Setter for GridLocationInWorld property
            obj.validateGridLocationInput(loc, 'GridLocationInWorld');
            obj.GridLocationInWorld = loc;
        end
    end
    
    methods(Access = protected)
        function xy = gridToWorldPrivate(obj, ij)
            %gridToWorldPrivate Convert grid indices to world coordinates
            
            % ROW-COL is YX, so convert it to XY
            ji = flip(ij, 2);
            
            % Cell index can only increment by 1
            halfCell = 0.5;
            
            % Y-axis starts at top, reverse it to start at bottom
            ji(:,2) = obj.GridSize(1)+1 - ji(:,2);
            
            % Compute the transform to converts index to world coordinate
            tform = ([halfCell, halfCell])/obj.Resolution - ...
                obj.GridLocationInWorld;
            
            % Apply transform
            xy = ji/obj.Resolution - repmat(tform,size(ji,1),1);
        end
        
        function ij = worldToGridPrivate(obj, xy)
            %worldToGridPrivate Convert world coordinates to grid indices
            
            % Compute all cells indices
            translatedPos = (xy - repmat(obj.GridLocationInWorld,size(xy,1),1));
            
            % Convert to ROW-COL which is YX
            translatedPosYX = flip(translatedPos, 2);
            
            ij = ceil(translatedPosYX*obj.Resolution);
            
            % GridLocation is always cell [1 1]
            originIdx = abs(translatedPosYX) < eps;
            ij(originIdx) = 1;
            
            % Adjust grid indices to start at top
            ij(:,1) = obj.GridSize(1)+1 - ij(:,1);
        end
    end
    
    %================================================================
    methods (Static, Access = protected)
        function pos = validatePosition(pos, xlimits, ylimits, fcnName, argName)
            %validatePosition Validate the world position column matrix
            
            % Validate the input format and type
            validateattributes(pos, {'numeric'}, ...
                {'real', 'nonnan', 'finite', 'nonempty', '2d', 'ncols', 2}, fcnName, argName);
            
            pos = double(pos);
            
            % Validate if the input is within the world limits
            maxpos = max(pos, [], 1);
            minpos = min(pos, [], 1);
            
            isOutside  = (minpos(1,1) < xlimits(1,1) || minpos(1,2) < ylimits(1,1) ...
                || maxpos(1,1) > xlimits(1,2) || maxpos(1,2) > ylimits(1,2));
            if isOutside
                if coder.target('MATLAB')
                    error(message('robotics:robotalgs:occgridcommon:CoordinateOutside', ...
                        num2str(xlimits(1,1),'%.2f'), num2str(xlimits(1,2),'%.2f'), ...
                        num2str(ylimits(1,1),'%.2f'), num2str(ylimits(1,2),'%.2f')));
                else
                    coder.internal.errorIf(isOutside, 'robotics:robotalgs:occgridcommon:CoordinateOutside', ...
                        coder.internal.num2str(xlimits(1,1)), coder.internal.num2str(xlimits(1,2)), ...
                        coder.internal.num2str(ylimits(1,1)), coder.internal.num2str(ylimits(1,2)));
                end
            end
        end
        
        function pos = validateGridIndices(pos, gridsize, fcnName, argName)
            %validateGridIndices Validate the grid indices column matrix
            
            % Validate the input format and type
            validateattributes(pos,{'numeric'}, ...
                {'integer', 'nonnan', 'positive', 'nonempty', '2d', 'ncols', 2}, ...
                fcnName, argName)
            
            validateattributes(gridsize,{'numeric'}, ...
                {'integer', 'nonnan', 'positive', 'nonempty', '2d', 'ncols', 2}, ...
                fcnName, argName)
            
            pos = double(pos);
            
            % Validate if the indices are within the limits
            maxpos = max(pos, [], 1);
            isOutside = (maxpos(1) > gridsize(1,1) || ...
                maxpos(2) > gridsize(1,2));
            if isOutside
                coder.internal.errorIf(isOutside,...
                    'robotics:robotalgs:occgridcommon:IndexExceedsDim', ...
                    gridsize(1,1), gridsize(1,2));
            end
        end
        
        function loc = validateGridLocationInput(loc, argName)
            % Validate the input format and type
            validateattributes(loc,{'numeric', 'logical'}, ...
                {'real', 'nonnan', 'finite', 'size', [1 2]}, ...
                argName)
            loc = double(loc);
        end
        
        function isGrid = parseOptionalFrameInput(cframe, fcnName)
            %parseOptionalFrameInput Input parser for optional string
            
            validCoordFrames = {'grid','world'};
            frame = validatestring(cframe,validCoordFrames,fcnName);
            isGrid = strcmp(frame,'grid');
        end
        
        function [frameValid,frame] = validateCoordFrames(cframe)
            %validateCoordFrames Validates the coordinate frame
            validCoordFrames = {'grid','world'};
            frame = validatestring(cframe,validCoordFrames);
            frameValid = any(frame);
        end
        
        function res = validateResolution(res, fcnName)
            %validateResolution Validates the resolution input
            validateattributes(res, ...
                {'numeric', 'logical'}, ...
                {'scalar', 'positive','nonnan','finite'},...
                fcnName,'RES');
            res = double(res);
        end
        
        function [M, N] = validateGridInput(M,N, fcnName)
            %validateGridInput Validates the grid inputs rows and columns
            validateattributes(M, ...
                {'numeric', 'logical'}, ...
                {'finite','real','positive','scalar','nonnan','integer'}, fcnName, 'M', 1);
            validateattributes(N, ...
                {'numeric', 'logical'}, ...
                {'scalar','real','positive','finite','nonnan','integer'}, fcnName, 'N', 2);
            M = double(M);
            N = double(N);
        end
        
        function [W, H] = validateWorldInput(W,H, fcnName)
            %validateWorldInput Validates the world inputs width and height
            validateattributes(W, ...
                {'numeric', 'logical'}, ...
                {'scalar','real','positive','nonnan', 'finite'}, fcnName, 'W', 1);
            validateattributes(H, ...
                {'numeric', 'logical'}, ...
                {'scalar','real','positive','nonnan', 'finite'}, fcnName, 'H', 2);
            W = double(W);
            H = double(H);
        end
        
        function radius = validateInflationRadius(inflationRad, ...
                resolution, isgrid, gridsize, argName)
            %validateInflationRadius Validate inflation radius
            
            % Validate that the input is double
            validateattributes(inflationRad, {'double'}, {'scalar','positive','nonnan', 'finite'}, 'inflate', argName);
            
            % Convert radius from meters to number of cells
            if isgrid
                radius = inflationRad;
            else
                radius = ceil(inflationRad*resolution);
            end
            
            % Validate if the radius is integer and not excessively large
            validateattributes(radius,{'double'},{'integer', '<=', max(gridsize)}, ...
                'inflate', argName);
        end
        
        function outPose = validatePose(inPose, xLimits, yLimits, fcnName, argName)
            %validatePose Validate robot pose
            
            validateattributes(inPose, {'numeric'}, {'real', 'nonnan', 'finite', 'vector', 'numel', 3}, fcnName, argName);
            outPose = double(inPose(:).');
            
            % Verify that pose is within the boundaries of the map
            validateattributes(outPose(1), {'numeric'}, {'>=', xLimits(1), '<=', xLimits(2)}, fcnName, argName);
            validateattributes(outPose(2), {'numeric'}, {'>=', yLimits(1), '<=', yLimits(2)}, fcnName, argName);
            outPose(3) = robotics.internal.wrapToPi(outPose(3));
        end
        
        function outAngles= validateAngles(inAngles, fcnName, argName)
            %validateAngles Validate laser angles
            
            validateattributes(inAngles, {'numeric'}, {'real', 'nonnan', 'finite', 'vector'}, fcnName, argName);
            outAngles = double(inAngles(:));
        end
        
        function outRanges = validateRanges(inRanges, fcnName, argName)
            %validateRanges Validate laser ranges
            %   NaN values are allowed, the method has to decide on what to
            %   do with NaN ranges.
            
            validateattributes(inRanges, {'numeric'}, {'real', 'vector', 'nonnegative'}, fcnName, argName);
            outRanges = double(inRanges(:));
        end
    end
    
    methods (Access = protected)
        function [axHandle, imageHandle] = showGrid(obj, mat, axHandle, isGrid)
            %showGrid Display the occupancy grid in a figure
            %   [AH, IH] = showGrid(OBJ, MAT, AXHANDLE, ISGRID) plots the
            %   matrix MAT using imshow function on the provided axes AXHANDLE
            %   and returns the axes handle AH and image handle IH. ISGRID
            %   is used to indicate if the axes label need to be in world units or
            %   grid indices.
            %
            % Calling sequence for derived class
            % [axHandle, isGrid] = obj.showInputParser(varargin{:});
            % imageHandle = showGrid(obj, grid, axHandle, isGrid)
            % title(axHandle, message('robotics:robotalgs:occgrid:FigureTitle').getString);
            
            % If axes not given, create an axes
            % The newplot function does the right thing
            if isempty(axHandle)
                axHandle = newplot;
            end
            
            % Make axes invisible before plotting
            axHandle.Visible = 'off';
            
            % Use gray colormap for correct visualization
            cmap = colormap(axHandle, 'gray');
            % Flip color map to make unoccupied (free) cells white
            cmap = flip(cmap);
            
            % Display the grid map
            imageHandle = imshow(mat, 'Parent', axHandle, ...
                'InitialMagnification', 'fit');
            colormap(axHandle,cmap);
            
            % Change the axes limits, X data and Y data to show world
            % coordinates or grid indices on the figure
            if isGrid
                % Get the grid size
                xgdata = [1, obj.GridSize(2)];
                ygdata = [1, obj.GridSize(1)];
                
                % Set XData and YData
                imageHandle.XData = xgdata;
                imageHandle.YData = ygdata;
                
                % Compute the grid limits
                xlimits = [0.5, obj.GridSize(2)+0.5];
                ylimits = [0.5, obj.GridSize(1)+0.5];
                
                xlabel(axHandle, ...
                    message('robotics:robotalgs:occgridcommon:FigureColLabel').getString);
                ylabel(axHandle, ...
                    message('robotics:robotalgs:occgridcommon:FigureRowLabel').getString);
                
                % Set the axes
                set(axHandle, 'YDir','reverse');
                
            else
                % Get the world limits
                xlimits = obj.XWorldLimits;
                ylimits = obj.YWorldLimits;
                
                correction = 1/(2*obj.Resolution);
                
                % Set XData and YData
                if (abs(xlimits(1)-xlimits(2)+2*correction) < eps)
                    % Special case when there is only one cell
                    imageHandle.XData = [xlimits(1), xlimits(2)];
                else
                    imageHandle.XData = [xlimits(1)+correction, xlimits(2)-correction];
                end
                
                if (abs(ylimits(1)-ylimits(2)+2*correction) < eps)
                    imageHandle.YData = [ylimits(2), ylimits(1)];
                else
                    imageHandle.YData = [ylimits(2)-correction, ylimits(1)+correction];
                end
                
                xlabel(axHandle, ...
                    'X');
                ylabel(axHandle, ...
                    'Y');
                
                % Set the axes
                set(axHandle, 'YDir','normal');
            end
            
            grid(axHandle, 'off');
            
            % Set XLim and YLim
            axHandle.XLim = xlimits;
            axHandle.YLim = ylimits;
            
            % Make axes visible
            axHandle.Visible = 'on';
        end
        
        function grid = inflateGrid(obj, grid, inflationRad, frame)
            %inflate Inflate the occupied positions by a given amount
            %   Internal function to inflate grid.
            
            isGrid = false;
            % If optional argument present then parse it separately
            if nargin > 3
                isGrid = obj.parseOptionalFrameInput(frame, 'inflate');
            end
            
            % Validate inflation radius and conversion to grid
            radius = obj.validateInflationRadius(inflationRad, obj.Resolution, ...
                isGrid, obj.GridSize, 'R');
            
            se = robotics.algs.internal.diskstrel(radius);
            grid = robotics.algs.internal.inflate(grid, se);
        end
        
        function index = getIndices(obj, inPos, isGrid, fcnName)
            %getIndices Validate and return grid indices from positions
            
            if isGrid               % Use grid indices
                pos = obj.validateGridIndices(inPos, obj.GridSize, fcnName, 'IJ');
            else                    % Use world coordinates
                pos = obj.validatePosition(inPos, obj.XWorldLimits, obj.YWorldLimits, fcnName, 'XY');
                % Convert to grid indices
                pos = obj.worldToGridPrivate(pos);
            end
            % X-Y is flipped because grid is row-major representation
            % index = sub2ind(size(obj.Grid), pos(:,2), pos(:,1));
            index = sub2ind(obj.GridSize, pos(:,1), pos(:,2));
        end
        
        function [axHandle,isGrid]= showInputParser(obj,varargin)
            %showInputParser Input parser for show function
            
            if ~coder.target('MATLAB')
                %  Code generation
                
                % Always throw error when calling show in
                % generated code
                coder.internal.errorIf(~coder.target('MATLAB'),...
                    'robotics:robotalgs:occgridcommon:GraphicsSupportCodegen','show');
            end
            
            defaultCoordFrame = 'world';
            
            p = inputParser;
            
            addOptional(p,'CoordinateFrame', defaultCoordFrame, @obj.validateCoordFrames);
            addParameter(p, 'Parent', [], @robotics.internal.validation.validateAxesHandle);
            
            parse(p, varargin{:});
            res = p.Results;
            axHandle = res.Parent;
            
            isGrid = obj.parseOptionalFrameInput(res.CoordinateFrame, 'show');
        end
        
        function [resolution, isGrid, mat, width, height, isMat] = ...
                parseConstructorInputs(obj, firstarg, varargin)
            %parseConstructorInputs Input parser for occupancy grid
            %constructors
            %   Parsing following input variations
            %       OccupancyGrid(P)
            %       OccupancyGrid(P, RES)
            %       OccupancyGrid(W, H)
            %       OccupancyGrid(W, H, RES)
            %       OccupancyGrid(W, H, RES, 'world')
            %       OccupancyGrid(M, N, RES, 'grid')
            
            className = obj.getClassName();
            
            % Decide parse path by checking the first input argument
            if isnumeric(firstarg) && isscalar(firstarg)  && ~isempty(varargin)
                isMat = false;
            else
                isMat = true;
            end
            
            % Parse optional inputs
            if isMat
                % Get required inputs
                mat = firstarg;
                % Set default values
                isGrid = false;
                resolution = 1;
                % Validate optional inputs
                % Ignore the implicit OccupancyGrid class object input
                switch nargin-1
                    case 1
                        % OccupancyGrid(P)
                        % Pass, use default optional values
                    case 2
                        % OccupancyGrid(P, RES)
                        % Parse the first var size input as 'Resolution'
                        res = obj.validateResolution(varargin{1}, className);
                        resolution = res;
                    otherwise
                        coder.internal.errorIf((nargin > 3), 'MATLAB:narginchk:tooManyInputs');
                end
            else
                % Get required inputs
                x = firstarg;
                y = varargin{1};
                % Set defaults
                resolution = 1;
                isGrid = false;
                % Validate optional inputs
                % Ignore the implicit OccupancyGrid class object input
                switch nargin-1
                    case 1
                        % This case never happens because this will lead to
                        % isMat = true
                    case 2
                        % OccupancyGrid(W, H)
                        % Pass, use default optional values
                    case 3
                        % OccupancyGrid(W, H, RES)
                        % Parse the second var size input as 'Resolution'
                        res = obj.validateResolution(varargin{2}, className);
                        resolution = res;
                    case 4
                        % OccupancyGrid(W, H, RES, 'world')
                        % OccupancyGrid(M, N, RES, 'grid')
                        % Parse the second var size input as 'Resolution'
                        res = obj.validateResolution(varargin{2}, className);
                        resolution = res;
                        % Parse the third var size input as
                        % 'CoordinateFrame'
                        validCoordFrames = {'grid','world'};
                        frame = validatestring(varargin{3},validCoordFrames,obj.getClassName(),'coordinate',4);
                        isGrid = strcmp(frame,'grid');
                    otherwise
                        coder.internal.errorIf((nargin > 5), 'MATLAB:narginchk:tooManyInputs');
                end
            end
            
            % Produce return values
            if isMat % Validate required inputs
                isGrid = false;
                obj.validateMatrixInput(mat);
                mat = obj.processMatrix(mat);
                columns = size(mat,2);
                rows = size(mat,1);
                width = columns/resolution;
                height = rows/resolution;
                
            elseif isGrid % Two scalar inputs with 'grid' option
                [x, y] = obj.validateGridInput(x,y, className);
                rows = x;
                columns = y;
                mat = obj.processMatrix(rows,columns);
                width = columns/resolution;
                height = rows/resolution;
                
            else  % Two scalar inputs with 'world' option which is also default
                [x, y] = obj.validateWorldInput(x,y, className);
                width = x;
                height = y;
                rows = ceil(height*resolution);
                columns = ceil(width*resolution);
                mat = obj.processMatrix(rows,columns);
            end
        end
    end
    
    methods (Abstract, Access = protected)
        mat = processMatrix(obj, varargin)
        %processMatrix Process matrix before returning
        %The derived class has to support two input patterns
        %   processMatrix(obj, rows, columns);
        %   processMatrix(obj, mat);
    end
    
    methods (Abstract, Static, Access = public)
        validateMatrixInput(mat)
        %validateMatrixInput Matrix input validation function
        
        className = getClassName()
        %getClassName Get name of the class for throwing errors
    end
end