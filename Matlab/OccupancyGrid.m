classdef (Sealed)OccupancyGrid <OccupancyGridBase
    %OCCUPANCYGRID Create an occupancy grid
    %   OCCUPANCYGRID creates a 2D occupancy grid map. Each cell has
    %   a value representing the probability of occupancy of that cell.
    %   Probability values close to 1 represent certainty that the workspace
    %   represented by the cell is occupied by an obstacle. Values close to 0
    %   represent certainty that the workspace represented by the cell is not
    %   occupied and is obstacle-free.
    %
    %   The probability values in the occupancy grid are stored with
    %   a precision of at least 1e-3 to reduce memory usage and allow creation of
    %   occupancy grid objects to represent large workspace. The minimum and
    %   maximum probability that can be represented are 0.001 and 0.999
    %   respectively.
    %
    %   MAP = robotics.OccupancyGrid(W, H) creates a 2D occupancy grid
    %   object representing a world space of width(W) and height(H) in
    %   meters. The default grid resolution is 1 cell per meter.
    %
    %   MAP = robotics.OccupancyGrid(W, H, RES) creates an OccupancyGrid
    %   object with resolution(RES) specified in cells per meter.
    %
    %   MAP = robotics.OccupancyGrid(M, N, RES, 'grid') creates an
    %   OccupancyGrid object and specifies a grid size of M rows and N columns.
    %   RES specifies the cells per meter resolution.
    %
    %   MAP = robotics.OccupancyGrid(P) creates an occupancy grid object
    %   from the values in the matrix, P. The size of the grid matches the
    %   matrix with each cell value interpreted from that matrix location.
    %   Matrix, P, may contain any numeric type with values between zero(0)
    %   and one(1).
    %
    %   MAP = robotics.OccupancyGrid(P, RES) creates an OccupancyGrid
    %   object from matrix, P, with RES specified in cells per meter.
    %
    %   OccupancyGrid properties:
    %       FreeThreshold           - Threshold to consider cells as obstacle-free
    %       OccupiedThreshold       - Threshold to consider cells as occupied
    %       ProbabilitySaturation   - Saturation limits on probability values as [min, max]
    %       GridSize                - Size of the grid in [rows, cols] (number of cells)
    %       Resolution              - Grid resolution in cells per meter
    %       XWorldLimits            - Minimum and maximum values of X
    %       YWorldLimits            - Minimum and maximum values of Y
    %       GridLocationInWorld     - Location of grid in world coordinates
    %
    %
    %   OccupancyGrid methods:
    %       checkOccupancy  - Check locations for free, occupied or unknown
    %       copy            - Create a copy of the object
    %       getOccupancy    - Get occupancy of a location
    %       grid2world      - Convert grid indices to world coordinates
    %       inflate         - Inflate each occupied grid location
    %       insertRay       - Insert rays from laser scan observation
    %       occupancyMatrix - Convert occupancy grid to double matrix
    %       raycast         - Compute cell indices along a ray
    %       rayIntersection - Compute map intersection points of rays
    %       setOccupancy    - Set occupancy of a location
    %       show            - Show grid values in a figure
    %       updateOccupancy - Integrate probability observation at a location
    %       world2grid      - Convert world coordinates to grid indices
    %
    %
    %   Example:
    %
    %       % Create a 2m x 2m empty map
    %       map = robotics.OccupancyGrid(2,2);
    %
    %       % Create a 10m x 10m empty map with resolution 20
    %       map = robotics.OccupancyGrid(10, 10, 20);
    %
    %       % Insert a laser scan in the occupancy grid
    %       ranges = 5*ones(100, 1);
    %       angles = linspace(-pi/2, pi/2, 100);
    %       insertRay(map, [5,5,0], ranges, angles, 20);
    %
    %       % Show occupancy grid in Graphics figure
    %       show(map);
    %
    %       % Create a map from a matrix with resolution 20
    %       p = eye(100)*0.5;
    %       map = robotics.OccupancyGrid(p, 20);
    %
    %       % Check occupancy of the world location (0.3, 0.2)
    %       value = getOccupancy(map, [0.3 0.2]);
    %
    %       % Set world position (1.5, 2.1) as occupied
    %       setOccupancy(map, [1.5 2.1], 0.8);
    %
    %       % Get the grid cell indices for world position (2.5, 2.1)
    %       ij = world2grid(map, [2.5 2.1]);
    %
    %       % Set the grid cell indices to unoccupied
    %       setOccupancy(map, [1 1], 0.2, 'grid');
    %
    %       % Integrate occupancy observation at a location
    %       updateOccupancy(map, [1 1], 0.7);
    %
    %       % Show occupancy grid in Graphics figure
    %       show(map);
    %
    %   See also robotics.MonteCarloLocalization, robotics.BinaryOccupancyGrid.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#codegen
    
    %%
    properties (Dependent)
        %OccupiedThreshold Probability threshold to consider cells as occupied
        %   A scalar representing the probability threshold above which
        %   cells are considered to be occupied. The OccupiedThreshold will be
        %   saturated based on ProbabilitySaturation property.
        %
        %   Default: 0.65
        OccupiedThreshold
        
        %FreeThreshold Probability threshold to consider cells as obstacle-free
        %   A scalar representing the probability threshold below which
        %   cells are considered to be obstacle-free. The FreeThreshold will be
        %   saturated based on ProbabilitySaturation property.
        %
        %   Default: 0.20
        FreeThreshold
        
        %ProbabilitySaturation Saturation values for probability
        %   A vector [MIN MAX] representing the probability saturation
        %   values. The probability values below MIN value will be saturated to
        %   MIN and above MAX values will be saturated to MAX. The MIN
        %   value cannot be below 0.001 and MAX value cannot be above 0.999.
        %
        %   Default: [0.001 0.999]
        ProbabilitySaturation
    end
    
    
    properties (Access = public)
        %Logodds Internal log-odds representation for probability
        Logodds
    end
    
    properties (Access = protected, Constant)
        %DefaultType Default datatype used for log-odds
        DefaultType = 'int16';
        
        %ProbMaxSaturation Maximum probability saturation possible
        ProbMaxSaturation = [0.001 0.999]
        
        %LookupTable Lookup table to convert between log-odds and probability
        LookupTable = createLookupTable();
        
        %LinSpaceInt16 The line space for all integer log-odds values
        LinSpaceInt16 = getIntLineSpace();
    end
    
    properties (Access = private)
        %FreeThresholdIntLogodds Integer log-odds values for FreeThreshold
        %   The log-odds value correspond to the default threshold of 0.2
        FreeThresholdIntLogodds
        
        %OccupiedThresholdIntLogodds Integer log-odds values for OccupiedThreshold
        %   The log-odds value correspond to the default threshold of 0.65
        OccupiedThresholdIntLogodds
    end
    
    
    properties (Access = public)
        %LogoddsHit Integer log-odds value for update on sensor hit
        %   This is the default log-odds value used to update grid cells
        %   where a hit is detected. This value represents probability
        %   value of 0.7
        LogoddsHit
        
        %LogoddsMiss Integer log-odds value for update on sensor miss
        %   This is the default log-odds value used to update grid cells
        %   where a miss is detected (i.e. laser ray passes through grid
        %   cells). This value represents probability value of 0.4
        LogoddsMiss
        
        %ProbSatIntLogodds Int log-odds for saturation
        %   These values correspond to default ProbabilitySaturation
        %   values of [0.001 0.999]
        ProbSatIntLogodds
    end
    
    methods
        function set.OccupiedThreshold(obj,threshold)
            
            freeThreshold = obj.intLogoddsToProb(obj.FreeThresholdIntLogodds);
            validateattributes(threshold, {'numeric'}, ...
                {'scalar', 'real', 'nonnan', 'finite', 'nonnegative'}, ...
                'OccupiedThreshold');
            
            logOddsThreshold = obj.probToIntLogodds(double(threshold));
            check = obj.FreeThresholdIntLogodds > logOddsThreshold;
            if check
                if coder.target('MATLAB')
                    error(message('robotics:robotalgs:occgrid:ThresholdOutsideBounds', 'OccupiedThreshold', ...
                        '>=', num2str(freeThreshold,'%.3f')));
                else
                    
                    coder.internal.errorIf(check, 'robotics:robotalgs:occgrid:ThresholdOutsideBounds', 'OccupiedThreshold', ...
                        '>=', coder.internal.num2str(freeThreshold));
                end
            end
            obj.OccupiedThresholdIntLogodds = logOddsThreshold;
        end
        
        function threshold = get.OccupiedThreshold(obj)
            threshold = double(obj.intLogoddsToProb(obj.OccupiedThresholdIntLogodds));
        end
        
        function set.FreeThreshold(obj,threshold)
            occupiedThreshold = obj.intLogoddsToProb(obj.OccupiedThresholdIntLogodds);
            validateattributes(threshold, {'numeric'}, ...
                {'scalar', 'real', 'nonnan', 'finite', 'nonnegative'}, ...
                'FreeThreshold');
            
            
            logOddsThreshold = obj.probToIntLogodds(double(threshold));
            check = obj.OccupiedThresholdIntLogodds < logOddsThreshold;
            if check
                if coder.target('MATLAB')
                    error(message('robotics:robotalgs:occgrid:ThresholdOutsideBounds', 'FreeThreshold', ...
                        '<=', num2str(occupiedThreshold,'%.3f')));
                else
                    
                    coder.internal.errorIf(check, 'robotics:robotalgs:occgrid:ThresholdOutsideBounds', 'FreeThreshold', ...
                        '<=', coder.internal.num2str(occupiedThreshold));
                end
            end
            obj.FreeThresholdIntLogodds = logOddsThreshold;
        end
        
        function threshold = get.FreeThreshold(obj)
            threshold = double(obj.intLogoddsToProb(obj.FreeThresholdIntLogodds));
        end
        
        function set.ProbabilitySaturation(obj, sat)
            % Run basic checks
            validateattributes(sat, {'numeric'}, ...
                {'vector', 'real', 'nonnan', 'numel', 2}, 'ProbabilitySaturation');
            
            % Check the limits
            sortedSat = sort(sat);
            
            validateattributes(sortedSat(1), {'numeric'}, {'>=',obj.ProbMaxSaturation(1), ...
                '<=',0.5}, ...
                'ProbabilitySaturation', 'lower saturation');
            
            validateattributes(sortedSat(2), {'numeric'}, {'>=',0.5, ...
                '<=',obj.ProbMaxSaturation(2)}, ...
                'ProbabilitySaturation', 'upper saturation');
            
            obj.ProbSatIntLogodds = obj.probToIntLogoddsMaxSat(double([sortedSat(1), sortedSat(2)]));
            obj.Logodds(obj.Logodds < obj.ProbSatIntLogodds(1)) = obj.ProbSatIntLogodds(1);
            obj.Logodds(obj.Logodds > obj.ProbSatIntLogodds(2)) = obj.ProbSatIntLogodds(2);
        end
        
        function sat = get.ProbabilitySaturation(obj)
            sat = double(obj.intLogoddsToProb(obj.ProbSatIntLogodds));
        end
        
        function obj = OccupancyGrid(varargin)
            %OccupancyGrid Constructor
            
            % Parse input arguments
            narginchk(1,4);
            [resolution, isGrid, mat, width, height, isMat]...
                = obj.parseConstructorInputs(varargin{:});
            
            % Assign properties with default values
            obj.FreeThresholdIntLogodds = obj.probToIntLogoddsMaxSat(0.2);
            obj.OccupiedThresholdIntLogodds = obj.probToIntLogoddsMaxSat(0.65);
            obj.LogoddsHit = obj.probToIntLogoddsMaxSat(0.7);
            obj.LogoddsMiss = obj.probToIntLogoddsMaxSat(0.4);
            obj.ProbSatIntLogodds = [intmin(obj.DefaultType), intmax(obj.DefaultType)];
            
            % Apply probability saturation to matrix
            mat(mat < obj.ProbSatIntLogodds(1)) = obj.ProbSatIntLogodds(1);
            mat(mat > obj.ProbSatIntLogodds(2)) = obj.ProbSatIntLogodds(2);
            
            obj.Resolution = resolution;
            
            % Construct grid from a matrix input
            if isMat
                obj.Logodds = mat;
                obj.GridSize = size(obj.Logodds);
                return;
            end
            
            % Construct empty grid from width and height
            if isGrid
                obj.Logodds = mat;
            else
                gridsize = size(mat);
                % Throw a warning if we round off the grid size
                if any(gridsize ~= ([height, width]*obj.Resolution))
                    coder.internal.warning(...
                        'robotics:robotalgs:occgridcommon:RoundoffWarning');
                end
                
                obj.Logodds = mat;
            end
            obj.GridSize = size(obj.Logodds);
        end
        
        function cpObj = copy(obj)
            %COPY Creates a copy of the object
            %   cpObj = COPY(obj) creates a deep copy of the
            %   Occupancy Grid object with the same properties.
            %
            %   Example:
            %       % Create an occupancy grid of 10m x 10m world representation
            %       map = robotics.OccupancyGrid(10, 10);
            %
            %       % Create a copy of the object
            %       cpObj = copy(map);
            %
            %       % Access the class methods from the new object
            %       setOccupancy(cpObj,[2 4],true);
            %
            %       % Delete the handle object
            %       delete(cpObj)
            %
            %   See also robotics.OccupancyGrid
            
            if isempty(obj)
                % This will not be encountered in code generation
                cpObj = OccupancyGrid.empty(0,1);
            else
                % Create a new object with the same properties
                cpObj = OccupancyGrid(obj.GridSize(1),...
                    obj.GridSize(2),obj.Resolution,'grid');
                
                % Assign the grid data to the new object handle
                cpObj.Logodds = obj.Logodds;
                cpObj.GridLocationInWorld = obj.GridLocationInWorld;
                cpObj.OccupiedThresholdIntLogodds = obj.OccupiedThresholdIntLogodds;
                cpObj.FreeThresholdIntLogodds = obj.FreeThresholdIntLogodds;
                cpObj.ProbSatIntLogodds = obj.ProbSatIntLogodds;
            end
        end
        
        function value = getOccupancy(obj, pos, frame)
            %getOccupancy Get occupancy value for one or more positions
            %   VAL = getOccupancy(MAP, XY) returns an N-by-1 array of
            %   occupancy values for N-by-2 array, XY. Each row of the
            %   array XY corresponds to a point with [X Y] world coordinates.
            %
            %   VAL = getOccupancy(MAP, IJ, 'grid') returns an N-by-1
            %   array of occupancy values for N-by-2 array IJ. Each row of
            %   the array IJ refers to a grid cell index [X,Y].
            %
            %   Example:
            %       % Create an occupancy grid and get occupancy
            %       % values for a position
            %       map = robotics.OccupancyGrid(10, 10);
            %
            %       % Get occupancy of the world coordinate (0, 0)
            %       value = getOccupancy(map, [0 0]);
            %
            %       % Get occupancy of multiple coordinates
            %       [X, Y] = meshgrid(0:0.5:5);
            %       values = getOccupancy(map, [X(:) Y(:)]);
            %
            %       % Get occupancy of the grid cell (1, 1)
            %       value = getOccupancy(map, [1 1], 'grid');
            %
            %       % Get occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       values = getOccupancy(map, [I(:) J(:)], 'grid');
            %
            %   See also robotics.OccupancyGrid, setOccupancy
            
            isGrid = false;
            % If optional argument present then parse it separately
            if nargin > 2
                isGrid = obj.parseOptionalFrameInput(frame, 'getOccupancy');
            end
            
            % Validate position or subscripts and convert it to indices
            indices = obj.getIndices(pos, isGrid, 'getOccupancy');
            value = double(obj.intLogoddsToProb(obj.Logodds(indices)));
        end
        
        function setOccupancy(obj, pos, value, frame)
            %setOccupancy Set occupancy value for one or more positions
            %   setOccupancy(MAP, XY, VAL) assigns the scalar occupancy
            %   value, VAL to each coordinate specified in the N-by-2 array,
            %   XY. Each row of the array XY corresponds to a point with
            %   [X Y] world coordinates.
            %
            %   setOccupancy(MAP, XY, VAL) assigns each element of the
            %   N-by-1 vector, VAL to the coordinate position of the
            %   corresponding row of the N-by-2 array, XY.
            %
            %   setOccupancy(MAP, IJ, VAL, 'grid') assigns occupancy values
            %   to the grid positions specified by each row of the N-by-2
            %   array, IJ, which refers to the [row, col] index from each row
            %   in the array.
            %
            %   Example:
            %       % Create an occupancy grid and set occupancy
            %       % values for a position
            %       map = robotics.OccupancyGrid(10, 10);
            %
            %       % Set occupancy of the world coordinate (0, 0)
            %       setOccupancy(map, [0 0], 0.2);
            %
            %       % Set occupancy of multiple coordinates
            %       [X, Y] = meshgrid(0:0.5:5);
            %       values = ones(numel(X),1)*0.65;
            %       setOccupancy(map, [X(:) Y(:)], values);
            %
            %       % Set occupancy of the grid cell (1, 1)
            %       setOccupancy(map, [1 1], 0.4, 'grid');
            %
            %       % Set occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       setOccupancy(map, [I(:) J(:)], 0.4, 'grid');
            %
            %       % Set occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       values = ones(numel(I),1)*0.8;
            %       setOccupancy(map, [I(:) J(:)], values, 'grid');
            %
            %   See also robotics.OccupancyGrid, getOccupancy
            
            
            isGrid = false;
            % If optional argument present then parse it separately
            if nargin > 3
                isGrid = obj.parseOptionalFrameInput(frame, 'setOccupancy');
            end
            
            % Validate values
            obj.validateOccupancyValues(value, size(pos, 1), 'setOccupancy', 'val');
            
            % Validate position or subscripts and convert it to indices
            indices = obj.getIndices(pos, isGrid, 'setOccupancy');
            
            obj.Logodds(indices) = probToIntLogodds(obj,double(value(:)));
        end
        
        function inflate(obj, varargin)
            %INFLATE Inflate the occupied positions by a given amount
            %   INFLATE(MAP, R) inflates each occupied position of the
            %   occupancy grid by at least R meters. Each cell of the
            %   occupancy grid is inflated by number of cells which is the
            %   closest integer higher than the value MAP.Resolution*R.
            %
            %   INFLATE(MAP, R, 'grid') inflates each cell of the
            %   occupancy grid by R cells.
            %
            %   Note that the inflate function does not inflate the
            %   positions past the limits of the grid.
            %
            %   Example:
            %       % Create an occupancy grid and inflate map
            %       mat = eye(100)*0.6;
            %       map = robotics.OccupancyGrid(mat);
            %
            %       % Create a copy of the map for inflation
            %       cpMap = copy(map);
            %
            %       % Inflate occupied cells using inflation radius in
            %       % meters
            %       inflate(cpMap, 0.1);
            %
            %       % Inflate occupied cells using inflation radius in
            %       % number of cells
            %       inflate(cpMap, 2, 'grid');
            %
            %   See also robotics.OccupancyGrid, copy
            
            narginchk(2,3);
            obj.Logodds = obj.inflateGrid(obj.Logodds, varargin{:});
        end
        
        function imageHandle = show(obj, varargin)
            %SHOW Display the occupancy grid in a figure
            %   SHOW(MAP) displays the MAP occupancy grid in the
            %   current axes with the axes labels representing the world
            %   coordinates.
            %
            %   SHOW(MAP, 'grid') displays the MAP occupancy grid in
            %   the current axes with the axes of the figure representing
            %   the grid indices.
            %
            %   HIMAGE = SHOW(MAP, ___) returns the handle to the image
            %   object created by show.
            %
            %   SHOW(MAP,___,Name,Value) provides additional options specified
            %   by one or more Name,Value pair arguments. Name must appear
            %   inside single quotes (''). You can specify several name-value
            %   pair arguments in any order as Name1,Value1,...,NameN,ValueN:
            %
            %       'Parent'        - Handle of an axes that specifies
            %                         the parent of the image object
            %                         created by show.
            %
            %   Example:
            %       % Create an occupancy grid and display
            %       map = robotics.OccupancyGrid(eye(5)*0.5);
            %
            %       % Display the occupancy with axes showing the world
            %       % coordinates
            %       imgHandle = show(map);
            %
            %       % Display the occupancy with axes showing the grid
            %       % indices
            %       imgHandle = show(map, 'grid');
            %
            %       % Display the occupancy with axes showing the world
            %       % coordinates and specify a parent axes
            %       fHandle = figure;
            %       aHandle = axes('Parent', fh);
            %       imgHandle = show(map, 'world', 'Parent', ah);
            %
            %   See also robotics.OccupancyGrid
            
            [axHandle, isGrid] = obj.showInputParser(varargin{:});
            [axHandle, imghandle] = showGrid(obj, obj.intLogoddsToProb(obj.Logodds), axHandle, isGrid);
            title(axHandle, ...
                'Platoon Based Autonomous Intersection Management');
            
            % Only return handle if user requested it.
            if nargout > 0
                imageHandle = imghandle;
            end
        end
        
        function occupied = checkOccupancy(obj, pos, frame)
            %checkOccupancy Check ternary occupancy status for one or more positions
            %   VAL = checkOccupancy(MAP, XY) returns an N-by-1 array of
            %   occupancy status using OccupiedThreshold and FreeThreshold,
            %   for N-by-2 array, XY. Each row of the array XY corresponds
            %   to a point with [X Y] world coordinates. Occupancy status
            %   of 0 refers to obstacle-free cells, 1 refers to occupied
            %   cells and -1 refers to unknown cells.
            %
            %   VAL = checkOccupancy(MAP, IJ, 'grid') returns an N-by-1
            %   array of occupancy status for N-by-2 array IJ. Each row of
            %   the array IJ refers to a grid cell index [X,Y].
            %
            %   Example:
            %       % Create an occupancy grid
            %       map = robotics.OccupancyGrid(10, 10);
            %
            %       % Check occupancy status of the world coordinate (0, 0)
            %       value = checkOccupancy(map, [0 0]);
            %
            %       % Check occupancy status of multiple coordinates
            %       [X, Y] = meshgrid(0:0.5:5);
            %       values = checkOccupancy(map, [X(:) Y(:)]);
            %
            %       % Check occupancy status of the grid cell (1, 1)
            %       value = checkOccupancy(map, [1 1], 'grid');
            %
            %       % Check occupancy status of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       values = checkOccupancy(map, [I(:) J(:)], 'grid');
            %
            %   See also robotics.OccupancyGrid, getOccupancy
            
            isGrid = false;
            % If optional argument present then parse it separately
            if nargin > 2
                isGrid = obj.parseOptionalFrameInput(frame, 'checkOccupancy');
            end
            
            % Validate position or subscripts and convert it to indices
            indices = obj.getIndices(pos, isGrid, 'checkOccupancy');
            value = obj.Logodds(indices);
            
            freeIdx = (value < obj.FreeThresholdIntLogodds);
            occIdx = (value > obj.OccupiedThresholdIntLogodds);
            
            occupied = ones(size(value))*(-1);
            occupied(freeIdx) = 0;
            occupied(occIdx) = 1;
        end
        
        function idx = world2grid(obj, pos)
            %WORLD2GRID Convert world coordinates to grid indices
            %   IJ = WORLD2GRID(MAP, XY) converts an N-by-2 array of world
            %   coordinates, XY, to an N-by-2 array of grid indices, IJ. The
            %   input, XY, is in [X Y] format. The output grid indices, IJ,
            %   are in [ROW COL] format.
            %
            %   Example:
            %       % Create an occupancy grid and convert world
            %       % coordinates to grid indices
            %       % Create a 10m x 10m world representation
            %       map = robotics.OccupancyGrid(10, 10);
            %
            %       % Get grid indices from world coordinates
            %       ij = world2grid(map, [0 0])
            %
            %       % Get grid indices from world coordinates
            %       [x y] = meshgrid(0:0.5:2);
            %       ij = world2grid(map, [x(:) y(:)])
            
            pos = obj.validatePosition(pos, obj.XWorldLimits, ...
                obj.YWorldLimits, 'world2grid', 'xy');
            
            % Convert world coordinate to grid indices
            idx = worldToGridPrivate(obj, pos);
        end
        
        function pos = grid2world(obj, idx)
            %GRID2WORLD Convert grid indices to world coordinates
            %   XY = GRID2WORLD(MAP, IJ) converts an N-by-2 array of grid
            %   indices, IJ, to an N-by-2 array of world coordinates, XY. The
            %   input grid indices, IJ, are in [ROW COL] format. The output,
            %   XY, is in [X Y] format.
            %
            %   Example:
            %       % Create an occupancy grid and convert grid
            %       % indices to world coordinates
            %       % Create a 10m x 10m world representation
            %       map = robotics.OccupancyGrid(10, 10);
            %
            %       % Get world coordinates from grid indices
            %       xy = grid2world(map, [1 1])
            %
            %       % Get world coordinates from grid indices
            %       [i j] = meshgrid(1:5);
            %       xy = world2grid(map, [i(:) j(:)])
            
            idx = obj.validateGridIndices(idx, obj.GridSize, ...
                'grid2world', 'IJ');
            
            % Convert grid index to world coordinate
            pos = gridToWorldPrivate(obj, idx);
        end
        
        function updateOccupancy(obj, pos, value, frame)
            %updateOccupancy Integrate occupancy value for one or more positions
            %   updateOccupancy(MAP, XY, OBS) probabilistically integrates
            %   the scalar observation OBS for each coordinate specified in the
            %   N-by-2 array, XY. Each row of the array XY corresponds to a
            %   point with [X Y] world coordinates. Default update values
            %   are used if OBS is logical. Update values are 0.7 and 0.4
            %   for true and false respectively. Alternatively, OBS can be
            %   of any numeric type with value between 0 and 1.
            %
            %   updateOccupancy(MAP, XY, OBS) probabilistically integrates
            %   each element of the N-by-1 vector, OBS with the coordinate
            %   position of the corresponding row of the N-by-2 array, XY.
            %
            %   updateOccupancy(MAP, IJ, OBS, 'grid') probabilistically
            %   integrates observation values to the grid positions
            %   specified by each row of the N-by-2 array, IJ, which refers
            %   to the [row, col] index from each row in the array.
            %
            %   Example:
            %       % Create an occupancy grid and update occupancy
            %       % values for a position
            %       map = robotics.OccupancyGrid(10, 10);
            %
            %       % Update occupancy of the world coordinate (0, 0)
            %       updateOccupancy(map, [0 0], true);
            %
            %       % Update occupancy of multiple coordinates
            %       [X, Y] = meshgrid(0:0.5:5);
            %       values = ones(numel(X),1)*0.65;
            %       updateOccupancy(map, [X(:) Y(:)], values);
            %
            %       % Update occupancy of the grid cell (1, 1)
            %       updateOccupancy(map, [1 1], false, 'grid');
            %
            %       % Update occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       updateOccupancy(map, [I(:) J(:)], 0.4, 'grid');
            %
            %       % Update occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       values = true(numel(I),1);
            %       updateOccupancy(map, [I(:) J(:)], values, 'grid');
            %
            %   See also robotics.OccupancyGrid, setOccupancy
            isGrid = false;
            
            % If optional argument present then parse it separately
            if nargin > 3
                isGrid = obj.parseOptionalFrameInput(frame, 'updateOccupancy');
            end
            
            % Validate values
            obj.validateOccupancyValues(value, size(pos, 1), 'updateOccupancy', 'val');
            
            % Validate position or subscripts and convert it to indices
            indices = obj.getIndices(pos, isGrid, 'updateOccupancy');
            
            if isscalar(value)
                value = cast(ones(size(pos, 1), 1)*value, 'like', value);
            end
            
            if islogical(value)
                updateValuesHit = obj.Logodds(indices(logical(value(:)))) + obj.LogoddsHit;
                updateValuesHit(updateValuesHit > obj.ProbSatIntLogodds(2)) = obj.ProbSatIntLogodds(2);
                obj.Logodds(indices(logical(value(:)))) = updateValuesHit;
                
                updateValuesMiss = obj.Logodds(indices(~logical(value(:)))) + obj.LogoddsMiss;
                updateValuesMiss(updateValuesMiss < obj.ProbSatIntLogodds(1)) = obj.ProbSatIntLogodds(1);
                obj.Logodds(indices(~logical(value(:)))) = updateValuesMiss;
            else
                updateValues = obj.Logodds(indices) + obj.probToIntLogoddsMaxSat(double(value(:)));
                updateValues(updateValues > obj.ProbSatIntLogodds(2)) = obj.ProbSatIntLogodds(2);
                updateValues(updateValues < obj.ProbSatIntLogodds(1)) = obj.ProbSatIntLogodds(1);
                obj.Logodds(indices) = updateValues;
            end
        end
        
        function collisionPt = rayIntersection(obj, pose, angles, maxRange, threshold)
            %rayIntersection Compute map intersection points of rays
            %   PTS = rayIntersection(MAP, POSE, ANGLES, MAXRANGE) returns
            %   collision points PTS in the world coordinate frame for
            %   rays emanating from POSE. PTS is an N-by-2 array of points.
            %   POSE is a 1-by-3 array of sensor pose [X Y THETA] in the world
            %   coordinate frame. ANGLES is an N-element vector of angles
            %   at which to get ray intersection points. MAXRANGE is a
            %   scalar representing the maximum range of the range sensor. If
            %   there is no collision up-to the maximum range then [NaN NaN]
            %   output is returned. By default, the OccupiedThreshold
            %   property in MAP is used to determine occupied cells.
            %
            %   PTS = rayIntersection(MAP, POSE, ANGLES, MAXRANGE, THRESHOLD)
            %   returns collision points PTS, where the THRESHOLD is used
            %   to determine the occupied cells. Any cell with a probability
            %   value greater or equal to THRESHOLD is considered occupied.
            %
            %   Example:
            %       % Create an occupancy grid
            %       map = robotics.OccupancyGrid(eye(10));
            %
            %       % Set occupancy of the world coordinate (5, 5)
            %       setOccupancy(map, [5 5], 0.5);
            %
            %       % Get collision points
            %       collisionPts = rayIntersection(map, [0,0,0], [pi/4, pi/6], 10);
            %
            %       % Visualize the collision points
            %       show(map);
            %       hold('on');
            %       plot(collisionPts(:,1),collisionPts(:,2) , '*')
            %
            %       % Get collision points with threshold value 0.4
            %       collisionPts = rayIntersection(map, [0,0,0], [pi/4, pi/6], 10, 0.4);
            %
            %       % Visualize the collision points
            %       plot(collisionPts(:,1),collisionPts(:,2) , '*')
            %
            %   See also robotics.OccupancyGrid, raycast
            
            narginchk(4,5);
            vPose = obj.validatePose(pose, obj.XWorldLimits, obj.YWorldLimits, 'rayIntersection', 'pose');
            vAngles = obj.validateAngles(angles, 'rayIntersection', 'angles');
            
            validateattributes(maxRange, {'numeric'}, ...
                {'real', 'nonnan', 'finite', 'scalar', 'positive'}, 'rayIntersection', 'maxrange');
            
            if nargin < 5
                grid = (obj.occupancyMatrix('ternary') == 1);
            else
                validateattributes(threshold, {'numeric'}, ...
                    {'real', 'nonnan', 'finite', 'scalar', '<=', 1, '>=', 0}, 'rayIntersection', 'threshold');
                grid = obj.occupancyMatrix > threshold;
            end
            
            [ranges, endPt] = robotics.algs.internal.calculateRanges(vPose, vAngles, double(maxRange), ...
                grid, obj.GridSize, obj.Resolution, obj.GridLocationInWorld);
            collisionPt = endPt;
            collisionPt(isnan(ranges), :) = nan;
        end
        
        function [endPts, middlePts] = raycast(obj, varargin)
            %RAYCAST Get cells along a ray
            %   [ENDPTS, MIDPTS] = RAYCAST(MAP, POSE, RANGE, ANGLE) returns
            %   cell indices of all cells traversed by a ray emanating from
            %   POSE at an angle ANGLE with length equal to RANGE. POSE is
            %   a 3-element vector representing robot pose [X, Y, THETA] in
            %   the world coordinate frame. ANGLE and RANGE are scalars.
            %   The ENDPTS are indices of cells touched by the
            %   end point of the ray. MIDPTS are all the cells touched by
            %   the ray excluding the ENDPTS.
            %
            %   [ENDPTS, MIDPTS] = RAYCAST(MAP, P1, P2) returns
            %   the cell indices of all cells between the line segment
            %   P1=[X1,Y1] to P2=[X2,Y2] in the world coordinate frame.
            %
            %   For faster insertion of range sensor data, use the insertRay
            %   method with an array of ranges or an array of end points.
            %
            %   Example:
            %       % Create an occupancy grid
            %       map = robotics.OccupancyGrid(10, 10, 20);
            %
            %       % compute cells along a ray
            %       [endPts, midPts] = raycast(map, [5,3,0], 4, pi/3);
            %
            %       % Change occupancy cells to visualize
            %       updateOccupancy(map, endPts, true, 'grid');
            %       updateOccupancy(map, midPts, false, 'grid');
            
            %       % Compute cells along a line segment
            %       [endPts, midPts] = raycast(map, [2,5], [6,8]);
            %
            %       % Change occupancy cells to visualize
            %       updateOccupancy(map, endPts, true, 'grid');
            %       updateOccupancy(map, midPts, false, 'grid');
            %
            %       % Visualize the raycast output
            %       show(map);
            %
            %   See also robotics.OccupancyGrid, insertRay
            
            narginchk(3,4);
            if nargin == 4
                pose = obj.validatePose(varargin{1}, obj.XWorldLimits, ...
                    obj.YWorldLimits, 'raycast', 'pose');
                validateattributes(varargin{2}, {'numeric'}, ...
                    {'real', 'nonnan', 'finite', 'scalar', 'nonnegative'}, 'raycast', 'range');
                validateattributes(varargin{3}, {'numeric'}, ...
                    {'real', 'nonnan', 'finite', 'scalar'}, 'raycast', 'angle');
                range = double(varargin{2});
                angle = double(varargin{3}) + pose(3);
                
                startPoint = pose(1:2);
                endPoint = [pose(1) + range*cos(angle), ...
                    pose(2) + range*sin(angle)];
            else
                startPoint = obj.validatePosition(varargin{1}, obj.XWorldLimits, ...
                    obj.YWorldLimits, 'raycast', 'StartPoint');
                validateattributes(varargin{1}, {'numeric'}, {'numel', 2}, ...
                    'raycast', 'p1');
                
                endPoint = obj.validatePosition(varargin{2}, obj.XWorldLimits, ...
                    obj.YWorldLimits, 'raycast', 'EndPoint');
                validateattributes(varargin{2}, {'numeric'}, {'numel', 2}, ...
                    'raycast', 'p2');
            end
            
            [endPts, middlePts] = robotics.algs.internal.raycastCells(startPoint, endPoint, ...
                obj.GridSize(1), obj.GridSize(2), obj.Resolution, obj.GridLocationInWorld);
        end
        
        function insertRay(obj, varargin)
            %insertRay Insert rays from laser scan observation
            %   insertRay(MAP, POSE, RANGES, ANGLES, MAXRANGE) inserts one
            %   or more range sensor observations in the occupancy grid.
            %   POSE is a 3-element vector representing sensor pose
            %   [X, Y, THETA] in world coordinate frame, RANGES and ANGLES
            %   are corresponding N-element vectors for range sensor
            %   readings, MAXRANGE is the maximum range of the sensor.
            %   The cells along the ray except the end points are observed
            %   as obstacle-free and updated with probability of 0.4.
            %   The cells touching the end point are observed as occupied
            %   and updated with probability of 0.7. NaN values in RANGES are
            %   ignored. RANGES above MAXRANGE are truncated and the end
            %   points are not updated for MAXRANGE readings.
            %
            %   insertRay(MAP, POSE, RANGES, ANGLES, MAXRANGE, INVMODEL)
            %   inserts the ray with update probabilities according to a
            %   2-element vector INVMODEL. The first element of
            %   INVMODEL is used to update obstacle-free observations and
            %   the second element is used to update occupied observations.
            %   Values in INVMODEL should be between 0 and 1.
            %
            %   insertRay(MAP, STARTPT, ENDPTS) inserts cells between the
            %   line segments STARTPT and ENDPTS. STARTPT is 2-element vector
            %   representing the start point [X,Y] in the world coordinate frame.
            %   ENDPTS is N-by-2 array of end points in the world coordinate
            %   frame. The cells along the line segment except the end points
            %   are updated with miss probability of 0.4 and the cells
            %   touching the end point are updated with hit probability of 0.7.
            %
            %   insertRay(MAP, STARTPT, ENDPTS, INVMODEL) inserts the
            %   line segment with update probabilities according to a
            %   1-by-2 or 2-by-1 array INVMODEL.
            %
            %   Example:
            %       % Create an occupancy grid
            %       map = robotics.OccupancyGrid(10,10,20);
            %
            %       % Insert two rays
            %       insertRay(map, [5,5,0], [5, 6], [pi/4, pi/6], 20);
            %
            %       % Insert rays with non default inverse model
            %       insertRay(map, [5,5,0], [5, 6], [pi/3, pi/2], 20, [0.3 0.8]);
            %
            %       % Visualize inserted rays
            %       show(map);
            %
            %       % Insert a line segment
            %       insertRay(map, [0,0], [3,3]);
            %
            %       % Visualize inserted ray
            %       show(map);
            %
            %   See also robotics.OccupancyGrid, raycast
            
            % Supported syntax
            %insertRay(ogmap, robotpose, ranges, angles, maxRange)
            %insertRay(ogmap, robotpose, ranges, angles, maxRange, inverseModel)
            
            %insertRay(ogmap, startPoint, endPoints)
            %insertRay(ogmap, startPoint, endPoints, inverseModel)
            
            gridSize = obj.GridSize;
            res = obj.Resolution;
            loc = obj.GridLocationInWorld;
            
            narginchk(3, 6);
            if nargin < 5
                % Cartesian input
                validateattributes(varargin{1}, {'numeric'}, ...
                    {'nonempty', 'numel', 2}, 'insertRay', 'startpt');
                startPt = [varargin{1}(1) varargin{1}(2)];
                startPt = obj.validatePosition(startPt, obj.XWorldLimits, ...
                    obj.YWorldLimits, 'insertRay', 'StartPoint');
                
                endPt = obj.validatePosition(varargin{2}, obj.XWorldLimits, ...
                    obj.YWorldLimits, 'insertRay', 'endpts');
            else
                % Polar input
                pose = obj.validatePose(varargin{1}, obj.XWorldLimits, ...
                    obj.YWorldLimits, 'insertRay', 'pose');
                maxRange = varargin{4};
                validateattributes(maxRange, {'numeric'}, ...
                    {'real', 'nonnan', 'finite', 'scalar', 'positive'}, 'insertRay', 'maxrange');
                
                ranges = obj.validateRanges(varargin{2}, 'insertRay', 'ranges');
                angles = obj.validateAngles(varargin{3}, 'insertRay', 'angles');
                isInvalidSize = (numel(ranges) ~= numel(angles));
                
                coder.internal.errorIf(isInvalidSize, ...
                    'robotics:robotalgs:occgridcommon:RangeAngleMismatch', 'ranges', 'angles');
                vRanges = ranges(~isnan(ranges(:)));
                vAngles = angles(~isnan(ranges(:)));
                vRanges = min(maxRange, vRanges);
                
                startPt = [pose(1) pose(2)];
                endPt = [pose(1) + vRanges.*cos(pose(3)+vAngles), ...
                    pose(2) + vRanges.*sin(pose(3)+vAngles)];
            end
            
            if nargin == 6 || nargin == 4
                % Validate inverse model
                validateattributes(varargin{end}, {'numeric'}, ...
                    {'real', 'nonnan', 'finite', 'nonempty', 'numel', 2}, 'insertRay', 'invmodel');
                
                invModel = varargin{end}(:)';
                
                if coder.target('MATLAB')
                    firstElementMsg = message('robotics:robotalgs:occgrid:FirstElement').getString;
                    secondElementMsg = message('robotics:robotalgs:occgrid:SecondElement').getString;
                else
                    % Hard code error string for code generation as coder
                    % does not support getting string from catalog.
                    firstElementMsg = 'first element of';
                    secondElementMsg = 'second element of';
                end
                
                validateattributes(invModel(1,1), {'numeric'}, ...
                    {'>=', 0, '<=', 0.5, 'scalar'}, 'insertRay', [firstElementMsg 'invModel']);
                validateattributes(invModel(1,2), {'numeric'}, ...
                    {'>=', 0.5, '<=', 1, 'scalar'}, 'insertRay', [secondElementMsg 'invModel']);
                
                inverseModelLogodds = obj.probToIntLogoddsMaxSat(double(invModel));
            else
                inverseModelLogodds = [obj.LogoddsMiss obj.LogoddsHit];
            end
            
            for i = 1:size(endPt, 1)
                [endPts, middlePts] = robotics.algs.internal.raycastCells(startPt, endPt(i,:), ...
                    gridSize(1), gridSize(2), res, loc);
                
                if ~isempty(middlePts)
                    mIndex = sub2ind(gridSize, middlePts(:,1), middlePts(:,2));
                    updateValuesMiss = obj.Logodds(mIndex) + inverseModelLogodds(1);
                    updateValuesMiss(updateValuesMiss < obj.ProbSatIntLogodds(1)) = obj.ProbSatIntLogodds(1);
                    obj.Logodds(mIndex) = updateValuesMiss;
                end
                
                % For max range reading do not add end point
                if nargin >= 5 && vRanges(i) >= maxRange
                    continue;
                end
                if ~isempty(endPts)
                    eIndex = sub2ind(gridSize, endPts(:,1), endPts(:,2));
                    updateValuesHit = obj.Logodds(eIndex) + inverseModelLogodds(2);
                    updateValuesHit(updateValuesHit > obj.ProbSatIntLogodds(2)) = obj.ProbSatIntLogodds(2);
                    obj.Logodds(eIndex) = updateValuesHit;
                end
            end
        end
        
        function mat = occupancyMatrix(obj, option)
            %OCCUPANCYMATRIX Export occupancy grid as a matrix
            %   MAT = OCCUPANCYMATRIX(MAP) returns probability values stored in the
            %   occupancy grid object as a matrix.
            %
            %   MAT = OCCUPANCYMATRIX(MAP, 'ternary') returns occupancy status of
            %   the each occupancy grid cell as a matrix. The
            %   OccupiedThreshold and FreeThreshold are used to determine
            %   obstacle-free and occupied cells. Value 0 refers to
            %   obstacle-free cell, 1 refers to occupied cell and -1 refers
            %   to unknown cell.
            %
            %   Example:
            %       % Create an occupancy grid
            %       inputMat = repmat(0.2:0.1:0.9, 8, 1);
            %       map = robotics.OccupancyGrid(inputMat);
            %
            %       % Export occupancy grid as a matrix
            %       mat = occupancyMatrix(map);
            %
            %       % Export occupancy grid as a ternary matrix
            %       mat = occupancyMatrix(map, 'ternary');
            %
            %   See also robotics.OccupancyGrid, getOccupancy
            
            narginchk(1,2);
            if nargin < 2
                mat = double(obj.intLogoddsToProb(obj.Logodds));
                return;
            end
            
            validStrings = {'Ternary'};
            validatestring(option, validStrings, 'occupancyMatrix');
            
            mat = -1*ones(size(obj.Logodds));
            
            occupied = (obj.Logodds > obj.OccupiedThresholdIntLogodds);
            free = (obj.Logodds < obj.FreeThresholdIntLogodds);
            mat(occupied) = 1;
            mat(free) = 0;
        end
    end
    
    %================================================================
    methods (Static, Access = {?matlab.unittest.TestCase, ...
            ?robotics.algs.internal.OccupancyGridBase})
        function logodds = probToLogodds(prob)
            %probToLogodds Get log-odds value from probabilities
            logodds = log(prob./(1-prob));
        end
        
        function probability = logoddsToProb(logodds)
            %logoddsToProb Get probabilities from log-odds values
            probability = 1 - 1./(1 + exp(logodds));
        end
        
        function maxSat = getMaxSaturation()
            %getMaxSaturation Get max saturation for testing purposes
            maxSat = robotics.OccupancyGrid.ProbMaxSaturation;
        end
    end
    
    
    methods (Access = {?matlab.unittest.TestCase, ...
            ?robotics.algs.internal.OccupancyGridBase})
        function logodds = probToIntLogodds(obj, prob)
            %probToIntLogodds Convert probability to int16 log-odds
            prob(prob < obj.ProbabilitySaturation(1)) = obj.ProbabilitySaturation(1);
            prob(prob > obj.ProbabilitySaturation(2)) = obj.ProbabilitySaturation(2);
            
            logodds = int16(interp1(obj.LookupTable, ...
                single(obj.LinSpaceInt16),prob,'nearest', 'extrap'));
        end
        
        function probability = intLogoddsToProb(obj, logodds)
            %intLogoddsToProb Convert int16 log-odds to probability
            
            probability = interp1(single(obj.LinSpaceInt16), ...
                obj.LookupTable, single(logodds),'nearest', 'extrap');
        end
        
        function logodds = probToIntLogoddsMaxSat(obj, prob)
            %probToIntLogoddsMaxSat Convert probability to int16 log-odds
            %   This method uses maximum possible saturation. Required for
            %   update occupancy.
            
            prob(prob < obj.ProbMaxSaturation(1)) = obj.ProbMaxSaturation(1);
            prob(prob > obj.ProbMaxSaturation(2)) = obj.ProbMaxSaturation(2);
            
            logodds = int16(interp1(obj.LookupTable, ...
                single(obj.LinSpaceInt16),prob,'nearest', 'extrap'));
        end
    end
    methods (Access = protected)
        function  mat = processMatrix(obj, varargin)
            %processMatrix Process construction input and create log-odds
            % Supports two input patterns:
            %   processMatrix(obj, rows,columns);
            %   processMatrix(obj, mat);
            if nargin == 3
                rows = varargin{1};
                columns = varargin{2};
                mat = zeros(rows, columns, obj.DefaultType);
            elseif nargin == 2
                % As probability saturation is not assigned, use max
                % saturation.
                mat = obj.probToIntLogoddsMaxSat(single(varargin{1}));
            end
        end
    end
    
    methods (Static, Access =public)
        function P = validateMatrixInput(P)
            %validateMatrixInput Validates the matrix input
            if islogical(P)
                validateattributes(P, {'logical'}, ...
                    {'2d', 'real', 'nonempty'}, ...
                    'OccupancyGrid', 'P', 1);
            else
                validateattributes(P, {'numeric','logical'}, ...
                    {'2d', 'real', 'nonempty','nonnan','<=',1,'>=',0}, ...
                    'OccupancyGrid', 'P', 1);
            end
        end
        
        function className = getClassName()
            className = 'OccupancyGrid';
        end
        
        function validateOccupancyValues(values, len, fcnName, argname)
            %validateOccupancyValues Validate occupancy value vector
            
            % check that the values are numbers in [0,1]
            if islogical(values)
                validateattributes(values, {'logical'}, ...
                    {'real','vector', 'nonempty'}, fcnName, argname);
            else
                validateattributes(values, {'numeric'}, ...
                    {'real','vector', 'nonnan', '<=',1,'>=',0, 'nonempty'}, fcnName, argname);
            end
            
            isSizeMismatch =  length(values) ~= 1 && length(values) ~= len;
            coder.internal.errorIf(isSizeMismatch,...
                'robotics:robotalgs:occgridcommon:InputSizeMismatch');
        end
    end
end


function linSpace = getIntLineSpace()
%getIntLineSpace Generate line space for integer log-odds
defaultType = OccupancyGrid.DefaultType;
linSpace = intmin(defaultType):intmax(defaultType);
end


function lookup = createLookupTable()
%createLookupTable Create lookup table for integer log-odds to probability
numOfPoints = abs(double(intmin(OccupancyGrid.DefaultType)))+...
    double(intmax(OccupancyGrid.DefaultType))+1;
logOddsLimits = OccupancyGrid.probToLogodds(OccupancyGrid.ProbMaxSaturation);
lookup = single(OccupancyGrid.logoddsToProb(...
    linspace(logOddsLimits(1), logOddsLimits(2), numOfPoints)));

end