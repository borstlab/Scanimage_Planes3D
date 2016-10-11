% makeGraphicsObjectMutable - Allow graphics objects to be moved/changed using the mouse.
%
% SYNTAX
%  makeGraphicsObjectMutable(hObject)
%  makeGraphicsObjectMutable(hObject, PropertyName, ProperyValue)
%    hObject - The handle to the graphics object to become mutable.
%
% PROPERTIES
%    callback - A function_handle, cell array (with a function_handle as the first element), or a string
%               to be executed after completion of a mutation event.
%               If callback is empty, it is ignored.
%    lockToAxes - Set to 1 to lock the axes or 0 to leave them free. Locked axes forces all changes in the object to stay within
%                 the current axes limits, and will not allow them to automatically stretch. The object can not be moved/stretched
%                 outside the current axes boundaries when lockToAxes is true.
%               Default: 0
%    lockX - Prevent motion/stretching in the X direction, setting this property to 1 enables it, while 0 disables it.
%            Default: 0
%    lockY - Prevent motion/stretching in the Y direction, setting this property to 1 enables it, while 0 disables it.
%            Default: 0
%    blockDrag - Do not allow dragging (left click functionality) of the object. Setting this property to 1 enables it, while 0 disables it.
%            Default: 0
%    blockStretch - Do not allow stretching (right click functionality) of the object. Setting this property to 1 enables it, while 0 disables it.
%            Default: 0
%    blockRectangleInversion - Do not allow rectangles to flip over. Setting this property to 1 enables it, while 0 disables it.
%            Default: 0
%    multiPointStretch - Allow stretch operations to move neighboring points, to a lesser degree, which results in a more natural feel. 1 to enable, 0 otherwise.
%            Default: 0
%    multiPointStretchSize - Indicates how many points, in each direction from the vertex of interest, to move during a stretch operation.
%                            If fractionalMultiPointStretch is 1, this will be a fraction of the total number of vertices (from 0-1), otherwise it is the number of vertices.
%            Default: 0.25
%    fractionaMultiPointStretch - Indicates how to interpret multiPointStretchSize.
%            Default: 1
%    forceClosedPolygon - Forces a line object to remain a closed polygon during multipoint stretching. Setting this property to 1 enables it, while 0 disables it.
%            Default: 0
%    passDeltaToCallback - When a callback function that takes arguments is provided, the total delta will be passed to the callback. Setting this property to 1 enables it, while 0 disables it.
%                          The format of the delta is dependent upon the type of the object (it is of the same size as the coordinates that compose the object).
%                          The delta value will be passed as the last argument to the callback, or second to last if 'passTypeToCallback' is enabled.
%            Default: 0
%    passTypeToCallback - Pass the type of mutation ('drag' or 'stretch') to the callback. Setting this property to 1 enables it, while 0 disables it.
%                         This argument will be passed as the last argument to the callback.
%            Default: 0
%
% USAGE
%  Once this function has been called on a graphics object it may then be manipulated using the mouse.
%  - A left click allows the user to drag the object around its current axes.
%  - A right click allows the user to "stretch" the object, which has different meanings, depending on the type of object:
%    'rectangle' - Do a "traditional" resize, by dragging the corner closest to the mouse at the click event.
%    'line' - Drag the closest vertex (datapoint) to the mouse at the click event, the connections to adjacent datapoints are maintained.
%
% NOTES
%   This function will coopt the object's 'ButtonDownFcn', and overwrite any values already there.
%
%   The 'ButtonDownFcn' and 'WindowButtonDownFcn' settings are disabled
%   during mutation to prevent mutation collisions (one must finish before the next can start).
%
%   Only 2D lines and rectangles are currently supported. More objects may be added later.
%
% EXAMPLE
%  % The following will create a figure with a line and rectangle object, with pseudo-callbacks, in order to play around.
%  % Recommending fiddling includes toggling the lockToAxes, lockX, and lockY properties.
%  f = figure;
%  r = rectangle('Position', [8 8 8 8], 'Tag', 'r');
%  l = line('XData', [1 5 9 12], 'YData', [3 6 21 35], 'Marker', 'o', 'Tag', 'l');
%  c = rectangle('Position', [2 20 5 5], 'Tag', 'c', 'Curvature', [1 1]);
%  makeGraphicsObjectMutable(r, 'Callback', 'disp(''RECTANGLE_MOVE_COMPLETE'')', 'lockToAxes', 1);
%  makeGraphicsObjectMutable(l, 'Callback', 'disp(''LINE_MOVE_COMPLETE'')', 'lockToAxes', 0);
%  makeGraphicsObjectMutable(c, 'Callback', 'disp(''CIRCLE_MOVE_COMPLETE'')', 'lockToAxes', 1);
%
% TODO (as of 2/19/05)
%  - Add support for patch objects.
%  - Fix "hopping" on rectangle inversion during stretching (hint: don't change coordinates 1 or 2 when about to invert).
%
% CHANGES
%  Tim O'Connor 3/3/05 - TO030305a: Fixed feval of cell array callback, it was missing the {:} expansion.
%  Tim O'Connor 3/4/05 - TO030405a: Select multiple vertices for closed polygonal lines.
%  Tim O'Connor 3/4/05 - TO030405b: Add a distinct tag to the glyph, 'makeGraphicsObjectMutableGlyph'.
%  Tim O'Connor 3/4/05 - TO030405c: Added the blockDrag, blockStretch, and blockRectangleInversion options.
%  Tim O'Connor 6/6/05 - TO060605A: Carry color information into the glyph, this may help contrast in some cases.
%  Tim O'Connor 8/31/06 - TO083106a: Corrected case in switch statement for property decoding in argument list
%  Tim O'Connor 7/07/07 - TO071807A: Make sure the HandleVisibility is On to allow glyph drawing. Do not rely on gcf/gca even though they should be fine.
%  Tim O'Connor 1/20/09 - TO012009B: Updated syntax in a few places to improve performance and/or remove Matlab editor warnings (ie. logical indexing, '&&' or '||', etc).
%  Tim O'Connor 1/20/09 - TO012009C: Implement a multi-point stretch, to get a little more natural feel.
%  Tim O'Connor 1/28/09 - TO012809B: Allow the passing of the total delta ('passDeltaToCallback' option) to the callback (needed to make deterministic movements at the higher level). Pass type as well.
%  Tim O'Connor 1/28/09 - TO012809C: Fully parameterize the multiPointStretch functionality (it had been hardcoded).
%  
% Created - Tim O'Connor 2/19/05
% Copyright - Timothy O'Connor 2005
function makeGraphicsObjectMutable(hObject, varargin)

callback = [];
options.lockToAxes = 0;
options.lockX = 0;
options.lockY = 0;
options.blockDrag = 0;%TO030405c
options.blockStretch = 0;%TO030405c
options.blockRectangleInversion = 0;%TO030405c
options.multiPointStretch = 0;%TO012009C
options.fractionalMultiPointStretch = 1;%TO012809C
options.multiPointStretchSize = 0.25;%TO012809C
options.forceClosedPolygon = 0;%TO012009C
options.passDelta = 0;%TO012809B
options.stackTrace = getStackTraceString;%TO012809B
options.passType = 0;%TO012809B
options.type = '';%TO012809B
if mod(length(varargin), 2) ~= 0
    error('Properties must come in name-value pairs.');
end
for i = 1 : 2 : length(varargin)
    switch lower(varargin{i})
        case 'callback'
            callback = varargin{i + 1};
        case 'locktoaxes'
            options.lockToAxes = varargin{i + 1};
        case 'lockx'
            options.lockX = varargin{i + 1};
        case 'locky'
            options.lockY = varargin{i + 1};
        case 'blockdrag' %TO083106a
            options.blockDrag = varargin{i + 1};%TO030405c
        case 'blockstretch' %TO083106a
            options.blockStretch = varargin{i + 1};%TO030405c
        case 'blockrectangleinversion'
            options.blockRectangleInversion = varargin{i + 1};%TO030405c
            error('NOT_YET_IMPLEMENTED');
        case 'multipointstretch' %TO012009C
            options.multiPointStretch = varargin{i + 1};
        case 'multipointstretchsize' %TO012809C
            options.multiPointStretchSize = varargin{i + 1};
        case 'fractionalmultipointstretch' %TO012809C
            options.fractionalMultiPointStretch = varargin{i + 1};
        case 'forceclosedpolygon' %TO012009C
            options.forceClosedPolygon = varargin{i + 1};
        case {'passdelta', 'passdeltatocallback'} %TO012809B
            options.passDelta = varargin{i + 1};
        case {'passtype', 'passtypetocallback'} %TO012809B
            options.passType = varargin{i + 1};
        otherwise
            error('Unrecognized property name: %s', varargin{i});
    end
end

if options.lockX && options.lockY %TO012009B
    error('Both X and Y motion have been locked, leaving no legal mutations.');
end

switch lower(class(callback))
    case 'function_handle'
        
    case 'cell'
        if ~isempty(callback)
            if ~strcmpi(class(callback{1}), 'function_handle')
                error('Cell array callbacks must have a function_handle as the first element: %s', class(callback{1}));
            end
        end
        
    case 'double'
        if ~isempty(callback)
            error('Callbacks may not be numbers.');
        end
        
    case 'char'
        if options.passDelta && options.passType %TO012809B
            error('Callback is a string, but the ''passDeltaToCallback'' and ''passTypeToCallback'' options require a callback that may take arguments (function_handle or a cell array that has a function_handle as its first element).');
        end
        if options.passDelta %TO012809B
            error('Callback is a string, but the ''passDeltaToCallback'' option requires a callback that may take arguments (function_handle or a cell array that has a function_handle as its first element).');
        end
        if options.passType %TO012809B
            error('Callback is a string, but the ''passTypeToCallback'' option requires a callback that may take arguments (function_handle or a cell array that has a function_handle as its first element).');
        end
        
    otherwise
        error('Invalid callback class: %s', class(callback));
end

set(hObject, 'ButtonDownFcn', {@executeGraphicsObjectMutation, callback, options});

return;

%------------------------------------------------
function executeGraphicsObjectMutation(hObject, eventdata, callback, options)

f = getParent(hObject, 'figure');
a = getParent(hObject, 'axes');

%TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
figureHandleVisibility = get(f, 'HandleVisibility');
axesHandleVisibility = get(a, 'HandleVisibility');
set(f, 'HandleVisibility', 'On');
set(a, 'HandleVisibility', 'On');

%Get the current location of the mouse.
currentPosition = get(a, 'currentPoint');
currentPosition = currentPosition(1, 1:2);%Only consider the (X, Y, 0) plane.

%Create the appropriate glyph.
switch lower(get(hObject, 'Type'))
    case 'rectangle'
        pos = get(hObject, 'Position');
        color = get(hObject, 'EdgeColor');%TO060605A
        nObject = rectangle('Position', pos, 'LineStyle', ':', 'EdgeColor', color, 'Curvature', get(hObject, 'Curvature'), 'Tag', 'makeGraphicsObjectMutableGlyph', 'Parent', a);%TO030405b %TO071807A
        %Find the vertex with the smallest Euclidean distance from the mouseclick, that's the one to be moved.
        vertex = findRectangleVertex(hObject);

    case 'line'
        if ~isempty(get(hObject, 'ZData'))
            error('Mutation of 3D lines is not currently supported.');
        end
        xData = get(hObject, 'XData');
        yData = get(hObject, 'YData');
        color = get(hObject, 'Color');%TO060605A
        nObject = line('XData', xData, 'YData', yData, 'LineStyle', ':', 'Color', color, 'Tag', 'makeGraphicsObjectMutableGlyph', 'Parent', a);%TO030405b %TO071807A
        %Find the datapoint with the smallest Euclidean distance from the mouseclick, that's the one to be moved.
        %Select multiple vertices for closed polygonal lines. TO030405a - Tim O'Connor 3/4/05
        distances = sqrt((currentPosition(1) - xData).^2 + (currentPosition(2) - yData).^2);
        mn = min(distances);%TO030405a - The min function only finds the first occurrence of a minima.
        vertex = find(distances == mn);%TO030405a - So, do a search for all matching points.
        
    otherwise
        %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
        set(a, 'HandleVisibility', axesHandleVisibility);
        set(f, 'HandleVisibility', figureHandleVisibility);
        error('Unsupported graphics object type for dragging.');
end

%Choose the appropriate mutation function.
switch lower(get(f, 'SelectionType'))
    case 'normal'
        %Drag
        options.type = 'drag';%TO012809B
        if options.blockDrag
            %TO030405c - New Option. -- Tim O'Connor 3/4/05
            delete(nObject);
            %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
            set(a, 'HandleVisibility', axesHandleVisibility);
            set(f, 'HandleVisibility', figureHandleVisibility);
            return;
        end
        motionFcn = {@dragUpdate, nObject, currentPosition, options};
        
    otherwise
        %Stretch
        options.type = 'stretch';%TO012809B
        if options.blockStretch
            %TO030405c - New Option. -- Tim O'Connor 3/4/05
            delete(nObject);
            %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
            set(a, 'HandleVisibility', axesHandleVisibility);
            set(f, 'HandleVisibility', figureHandleVisibility);
            return;
        end
        motionFcn = {@stretchUpdate, nObject, currentPosition, options, vertex};
end

axesButtonDownFcn = get(a, 'ButtonDownFcn');
objectButtonDownFcn = get(hObject, 'ButtonDownFcn');
windowButtonDownFcn = get(f, 'WindowButtonDownFcn');
windowButtonMotionFcn = get(f, 'WindowButtonMotionFcn');
windowButtonUpFcn_ = get(f, 'WindowButtonUpFcn');
doubleBuffer = get(f, 'DoubleBuffer');

%All this will get passed into the completion function ('WindowButtonUpFcn') to restore the original properties.
%To complete the mutation the original handle, the glyph handle, and the callback are required.
finishFcn = {@windowButtonUpFcn, hObject, nObject, callback, objectButtonDownFcn, axesButtonDownFcn, windowButtonMotionFcn, windowButtonUpFcn_, windowButtonDownFcn, doubleBuffer, options};%TO012809B

%Set up to track the motion.
set(a, 'ButtonDownFcn', finishFcn);
set(f, 'WindowButtonMotionFcn', motionFcn, 'DoubleBuffer', 'On', ...
            'WindowButtonUpFcn', finishFcn, 'WindowButtonDownFcn', finishFcn);

%TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
set(a, 'HandleVisibility', axesHandleVisibility);
set(f, 'HandleVisibility', figureHandleVisibility);

return;


%------------------------------------------------
%Update the glyph position.
function dragUpdate(figHandle, eventdata, nObject, lastPosition, options)

%gcf and gca should be well defined in this case, since we're working under a mouse click.
%TO071807A - In Matlab R2007a, gcf and gca screw things up, because Matlab is retarded.
% f = gcf;
% a = gca;
f = figHandle;
%TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
figureHandleVisibility = get(f, 'HandleVisibility');
set(f, 'HandleVisibility', 'On');
a = getParent(nObject, 'axes');
axesHandleVisibility = get(a, 'HandleVisibility');
set(a, 'HandleVisibility', 'On');

currentPosition = get(a, 'CurrentPoint');
if isempty(currentPosition)
    %No initial click --> This function shouldn't have been called.
    resolveErroneousWindowButtonMotionFcn(f, 'dragUpdate');
    %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
    set(a, 'HandleVisibility', axesHandleVisibility);
    set(f, 'HandleVisibility', figureHandleVisibility);
    return;
end
currentPosition = currentPosition(1, 1:2);%Only consider the (X, Y, 0) plane.

delta = currentPosition - lastPosition;
if options.lockX
    delta(1) = 0;
end
if options.lockY
    delta(2) = 0;
end

switch lower(get(nObject, 'Type'))
    case 'rectangle'
        pos = get(nObject, 'Position');
        pos(1:2) = pos(1:2) + delta;
        if options.lockToAxes
            xLim = get(a, 'XLim');
            yLim = get(a, 'YLim');
            if pos(1) + pos(3) > xLim(2)
                pos(1) = xLim(2) - pos(3);
            elseif pos(1) < xLim(1)
                pos(1) = xLim(1);
            end
            if pos(2) + pos(4) > yLim(2)
                pos(2) = yLim(2) - pos(4);
            elseif pos(2) < yLim(1)
                pos(2) = yLim(1);
            end            
        end
        set(nObject, 'Position', pos);
        
    case 'line'
        xData = get(nObject, 'XData') + delta(1);
        yData = get(nObject, 'YData') + delta(2);
        if options.lockToAxes
            xLim = get(a, 'XLim');
            yLim = get(a, 'YLim');
            if any(xData < xLim(1)) || any(xData > xLim(2))
                %This move is blocked by the axes limits.
                xData = get(nObject, 'XData');
            end
            if any(yData < yLim(1)) || any(yData > yLim(2))
                %This move is blocked by the axes limits.
                yData = get(nObject, 'YData');
            end
            xData(xData < xLim(1)) = xLim(1);%TO012009B
            xData(xData > xLim(2)) = xLim(2);%TO012009B
            yData(yData < yLim(1)) = xLim(1);%TO012009B
            yData(yData > yLim(2)) = xLim(2);%TO012009B
        end
        set(nObject, 'XData', xData);
        set(nObject, 'YData', yData);

    otherwise
        %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
        set(a, 'HandleVisibility', axesHandleVisibility);
        set(f, 'HandleVisibility', figureHandleVisibility);
        error('Unsupported type for dragging: %s', get(nObject, 'Type'));
end

%Update the 'lastPosition' variable in the callback.
set(f, 'WindowButtonMotionFcn', {@dragUpdate, nObject, currentPosition, options});

drawnow expose;%TO042309A - Not using expose can cause C-spawned events to fire out of order.

%TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
set(a, 'HandleVisibility', axesHandleVisibility);
set(f, 'HandleVisibility', figureHandleVisibility);

return;

%------------------------------------------------
% Update the glyph position.
%
% NOTES
%  This function has a tendency to cause "hops" when the rectangle gets "inverted".
%  It's unclear, at this moment, how to resolve this. -- Tim O'Connor 2/19/05
function stretchUpdate(figHandle, eventdata, nObject, lastPosition, options, vertex)

%gcf and gca should be well defined in this case, since we're working under a mouse click.
%TO071807A - In Matlab R2007a, gcf and gca screw things up, because Matlab is retarded.
% f = gcf;
% a = gca;
f = figHandle;
%TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
figureHandleVisibility = get(f, 'HandleVisibility');
set(f, 'HandleVisibility', 'On');
a = getParent(nObject, 'axes');
axesHandleVisibility = get(a, 'HandleVisibility');
set(a, 'HandleVisibility', 'On');

currentPosition = get(a, 'currentPoint');
if isempty(currentPosition)
    % No initial click --> This function shouldn't have been called.
    resolveErroneousWindowButtonMotionFcn(f, 'stretchUpdate');
    %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
    set(a, 'HandleVisibility', axesHandleVisibility);
    set(f, 'HandleVisibility', figureHandleVisibility);
    return;
end
currentPosition = currentPosition(1, 1:2);%Only consider the (X, Y, 0) plane.

delta = currentPosition - lastPosition;
if options.lockX
    delta(1) = 0;
end
if options.lockY
    delta(2) = 0;
end

type = lower(get(nObject, 'Type'));
if options.blockRectangleInversion && strcmp(type, 'rectangle') %TO012009B
    pos = get(nObject, 'Position');
    switch vertex
        case 1
            if delta(1) == pos(3) || delta(2) == pos(4) %TO012009B
                %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
                set(a, 'HandleVisibility', axesHandleVisibility);
                set(f, 'HandleVisibility', figureHandleVisibility);
                return;
            end
        case 2
            if delta(1) == -pos(3) || delta(2) == pos(4) %TO012009B
                %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
                set(a, 'HandleVisibility', axesHandleVisibility);
                set(f, 'HandleVisibility', figureHandleVisibility);
                return;
            end
        case 3
            if delta(1) == pos(3) || delta(2) == -pos(4) %TO012009B
                %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
                set(a, 'HandleVisibility', axesHandleVisibility);
                set(f, 'HandleVisibility', figureHandleVisibility);
                return;
            end
        case 4
            if delta(1) == -pos(3) || delta(2) == -pos(4) %TO012009B
                %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
                set(a, 'HandleVisibility', axesHandleVisibility);
                set(f, 'HandleVisibility', figureHandleVisibility);
                return;
            end
    end
end

switch type
    case 'rectangle'
        pos = get(nObject, 'Position');
        switch vertex
            case 1
                pos(1) = pos(1) + delta(1);
                pos(2) = pos(2) + delta(2);
                pos(3) = pos(3) - delta(1);
                pos(4) = pos(4) - delta(2);
            case 2
                pos(2) = pos(2) + delta(2);
                pos(3) = pos(3) + delta(1);
                pos(4) = pos(4) - delta(2);
            case 3
                pos(1) = pos(1) + delta(1);
                pos(3) = pos(3) - delta(1);
                pos(4) = pos(4) + delta(2);
            case 4
                pos(3) = pos(3) + delta(1);
                pos(4) = pos(4) + delta(2);
            otherwise
                %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
                set(a, 'HandleVisibility', axesHandleVisibility);
                set(f, 'HandleVisibility', figureHandleVisibility);
                error('Illegal rectangle vertex: %s', num2str(vertex));
        end
        if options.lockToAxes
            xLim = get(a, 'XLim');
            yLim = get(a, 'YLim');
            if pos(1) + pos(3) > xLim(2)
                pos(1) = xLim(2) - pos(3);
            elseif pos(1) < xLim(1)
                pos(1) = xLim(1);
            end
            if pos(2) + pos(4) > yLim(2)
                pos(2) = yLim(2) - pos(4);
            elseif pos(2) < yLim(1)
                pos(2) = yLim(1);
            end            
        end
        
        if pos(3) == 0
            %This is an illegal state, let them keep moving, or take the previous value.
            %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
            set(a, 'HandleVisibility', axesHandleVisibility);
            set(f, 'HandleVisibility', figureHandleVisibility);
            return;
        elseif pos(3) < 0
            pos(1) = pos(1) - pos(3);
            pos(3) = abs(pos(3));
            switch vertex
                case 1
                   vertex = 2;
                case 2
                    vertex = 1;
                case 3
                    vertex = 4;
                case 4
                    vertex = 3;
            end
        end
        if pos(4) == 0
            %This is an illegal state, let them keep moving, or take the previous value.
            %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
            set(a, 'HandleVisibility', axesHandleVisibility);
            set(f, 'HandleVisibility', figureHandleVisibility);
            return;
        elseif pos(4) < 0
            pos(2) = pos(2) - pos(4);
            pos(4) = abs(pos(4));
            switch vertex
                case 1
                   vertex = 3;
                case 2
                    vertex = 4;
                case 3
                    vertex = 1;
                case 4
                    vertex = 2;
            end
            
        end
        set(nObject, 'Position', pos);
        
    case 'line'
        xData = get(nObject, 'XData');
        xData(vertex) = xData(vertex) + delta(1);
        yData = get(nObject, 'YData');
        yData(vertex) = yData(vertex) + delta(2);
        %TO012009C - Implement a multi-point stretch, to get a little more natural feel. -- Tim O'Connor 1/20/09
        %TO012809C - Fully parameterize and vectorize the multiPointStretch functionality (it had been hardcoded initially). -- Tim O'Connor 1/28/09
        %TO031309C - TODO - FIX_ME - Multipoint-stretch is not fully parameterized, but I need this working smoothly, so kluge it for now.
        if options.multiPointStretch
% options.multiPointStretchSize = 0.75;
            if options.fractionalMultiPointStretch
                numPoints = round(options.multiPointStretchSize * length(xData) - 1);
            else
                numPoints = options.multiPointStretchSize;
            end
            if numPoints > length(xData) - 1
                warning('Number of vertices requested to be involved in stretch operation (%s) is more than the number of vertices available (%s).\n%s', num2str(numPoints), num2str(length(xData) - 1), options.stackTrace);
                numPoints = length(xData) - 1;
            end
%             simpleDistances = min(abs(vertex - (1 : length(xData))), abs(1:length(xData) - vertex));
%             % simpleDistances(simpleDistances < 0) = vertex + abs(simpleDistances(simpleDistances < 0));
%             wrappedDistances = abs(simpleDistances + (length(xData) - vertex));
%             backWrappedDistances = abs(simpleDistances + length(xData));
%             distancesFromVertex = min(min(simpleDistances, wrappedDistances), backWrappedDistances);
% % fprintf(1, 'vertex: %s\n', num2str(vertex));
% % fprintf(1, 'simpleDistances:      %s\n', mat2str(simpleDistances));
% % fprintf(1, 'wrappedDistances:     %s\n', mat2str(wrappedDistances));
% % fprintf(1, 'backWrappedDistances: %s\n', mat2str(backWrappedDistances));
% % fprintf(1, 'distancesFromVertex:  %s\n\n', mat2str(distancesFromVertex));
%             stretchableIndices = find(distancesFromVertex < numPoints);
            % stretchableIndices = [-round(0.5 * length(xData)) : round(0.5 * length(xData))];
            % stretchableIndices = stretchableIndices(stretchableIndices ~= 0);
            stretchableIndices = [-3, -2, -1, 1, 2, 3];
            stretchableIndices = stretchableIndices + vertex;
            stretchableIndices = stretchableIndices(stretchableIndices <= length(xData));
            stretchableIndices = stretchableIndices(stretchableIndices >= 1);
%             xData(stretchableIndices) = xData(stretchableIndices) + delta(1) * 1 ./ distancesFromVertex(stretchableIndices);
%             yData(stretchableIndices) = yData(stretchableIndices) + delta(2) * 1 ./ distancesFromVertex(stretchableIndices);
            xData(stretchableIndices) = xData(stretchableIndices) + delta(1) .* [0.25, 0.5, 0.75, 0.75, 0.5, 0.25];
            yData(stretchableIndices) = yData(stretchableIndices) + delta(2) .* [0.25, 0.5, 0.75, 0.75, 0.5, 0.25];
            if options.forceClosedPolygon
                if xData(1) ~= xData(end) || yData(1) ~= yData(end)
                    xData(end) = xData(1);
                    yData(end) = yData(1);
                end
            end
        end
        if options.lockToAxes
            xLim = get(a, 'XLim');
            yLim = get(a, 'YLim');
            xData(xData < xLim(1)) = xLim(1);%TO012009B
            xData(xData > xLim(2)) = xLim(2);%TO012009B
            yData(yData < yLim(1)) = xLim(1);%TO012009B
            yData(yData > yLim(2)) = xLim(2);%TO012009B
        end
        set(nObject, 'XData', xData, 'YData', yData);

    otherwise
        %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
        set(a, 'HandleVisibility', axesHandleVisibility);
        set(f, 'HandleVisibility', figureHandleVisibility);
        error('Unsupported type for dragging: %s', get(nObject, 'Type'));
end

%Update the 'lastPosition' variable in the callback.
set(f, 'WindowButtonMotionFcn', {@stretchUpdate, nObject, currentPosition, options, vertex});

drawnow expose;%TO042309A - Not using expose can cause C-spawned events to fire out of order.

%TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
set(a, 'HandleVisibility', axesHandleVisibility);
set(f, 'HandleVisibility', figureHandleVisibility);

return;

%------------------------------------------------
% Reset the figure/axes properties to their initial values.
% Adjust the original object to match the glyph.
% Execute any necessary notifications (callbacks).
% TO012809B - Take options as an argument now, to enable passDelta functionality.
function windowButtonUpFcn(figHandle, eventdata, hObject, nObject, callback, objectButtonDownFcn, axesButtonDownFcn, windowButtonMotionFcn, windowButtonUpFcn, windowButtonDownFcn, doubleBuffer, options)

%TO071807A - In Matlab R2007a, gcf and gca screw things up, because Matlab is retarded.
% f = gcf;
% a = gca;
f = figHandle;
%TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
figureHandleVisibility = get(f, 'HandleVisibility');
set(f, 'HandleVisibility', 'On');
a = getParent(nObject, 'axes');
axesHandleVisibility = get(a, 'HandleVisibility');
set(a, 'HandleVisibility', 'On');

%Revert to the original settings.
set(hObject, 'ButtonDownFcn', objectButtonDownFcn);
set(a, 'ButtonDownFcn', axesButtonDownFcn);
set(f, 'WindowButtonMotionFcn', windowButtonMotionFcn, 'DoubleBuffer', doubleBuffer, ...
            'WindowButtonUpFcn', windowButtonUpFcn, 'WindowButtonDownFcn', windowButtonDownFcn);

switch lower(get(nObject, 'Type'))
    case 'rectangle'
        if options.passDelta %TO012809B
            delta = get(nObject, 'Position') - get(hObject, 'Position');
        end
        set(hObject, 'Position', get(nObject, 'Position'));

    case 'line'
        if options.passDelta %TO012809B
            delta = cat(1, get(nObject, 'XData') - get(hObject, 'XData'), get(nObject, 'YData') - get(hObject, 'YData'));
        end
        set(hObject, 'XData', get(nObject, 'XData'));
        set(hObject, 'YData', get(nObject, 'YData'));

    otherwise
        %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
        set(a, 'HandleVisibility', axesHandleVisibility);
        set(f, 'HandleVisibility', figureHandleVisibility);
        error('Unsupported type for dragging: %s', get(nObject, 'Type'));
end

delete(nObject);

if ~isempty(callback)
    switch lower(class(callback))
        case 'function_handle'
            if options.passDelta && options.passType
                feval(callback, delta, options.type);
            elseif options.passDelta
                feval(callback, delta);
            elseif options.passType
                feval(callback, options.type);
            else
                feval(callback);
            end
            
        case 'cell'
            if ~strcmpi(class(callback{1}), 'function_handle')
                error('Cell array callbacks must have a function_handle as the first element: %s', class(callback{1}));
            end
            %TO030305a - Fixed feval of cell array callback, it was missing the {:} expansion. - Tim O'Connor 3/3/05
            if options.passDelta && options.passType
                feval(callback{:}, delta, options.type);
            elseif options.passDelta
                feval(callback{:}, delta);
            elseif options.passType
                feval(callback{:}, options.type);
            else
                feval(callback{:});
            end
            
        case 'char'
            if options.passDelta && options.passType %TO012809B
                warning('Requested ''passDeltaToCallback''/''passDeltaToCallback'' can not be performed because the callback is a string.\n%s\n', options.stackTrace);
            end
            if options.passDelta %TO012809B
                warning('Requested ''passDeltaToCallback'' can not be performed because the callback is a string.\n%s\n', options.stackTrace);
            end
            if options.passType %TO012809B
                warning('Requested ''passTypeToCallback'' can not be performed because the callback is a string.\n%s\n', options.stackTrace);
            end
            eval(callback);
            
        otherwise
            %TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
            set(a, 'HandleVisibility', axesHandleVisibility);
            set(f, 'HandleVisibility', figureHandleVisibility);
            error('Invalid callback class: %s\n%s', class(callback), options.stackTrace);
    end
end

%TO071807A - Make sure the HandleVisibility is On to allow glyph drawing.
set(a, 'HandleVisibility', axesHandleVisibility);
set(f, 'HandleVisibility', figureHandleVisibility);

return;

%------------------------------------------------
%Watch out for broken operations, if there's no CurrentPoint defined, nothing should be dragging.
%Verify that the proper WindowButtonUpFcn is defined and, if so, execute it.
%If things are still not fixed, at least remove the WindowButtonMotionFcn.
%Issue an appropriate warning.
function resolveErroneousWindowButtonMotionFcn(f, functionName)

resolved = 0;
corrupted = 0;

windowButtonUpFcn = get(f, 'WindowButtonUpFcn');

if iscell(windowButtonUpFcn) %TO012009B
    if ~isempty(windowButtonUpFcn)
        if strcmpi(class(windowButtonUpFcn{1}), 'function_handle')
            if strcmpi(func2str(windowButtonUpFcn{1}), 'windowButtonUpFcn')
                try
                    feval(windowButtonUpFcn);
                    resolved = 1;
                catch
                    warning('The WindowButtonUpFcn for the current figure ''%s'' has encountered an error: %s', num2str(f), lasterr);
                end
            end
        end
    end
end

windowButtonMotionFcn = get(f, 'WindowButtonMotionFcn');
if ~isempty(windowButtonMotionFcn)
    if iscell(windowButtonUpFcn) %TO012009B
        if ~isempty(windowButtonUpFcn)
            if strcmpi(class(windowButtonUpFcn{1}), 'function_handle')
                if strcmpi(func2str(windowButtonUpFcn{1}), functionName)
                    %Hmm, still not cleaned up.
                    set(f, 'WindowButtonMotionFcn', '');
                    resolved = 1;
                    corrupted = 1;
                end
            end
        end
    end
end

if resolved
    if ~corrupted
        warning('The WindowButtonMotionFcn for the current figure ''%s'' has been unexpectedly executed.\nThis problem may have been automatically resolved.', get(f, 'Name'));%TO012009B
    else
        warning('The WindowButtonMotionFcn for the current figure ''%s'' has been unexpectedly executed.\nThis problem may have been automatically resolved.\nThe figure''s properties may be corrupted.', get(f, 'Name'));%TO012009B
    end
else
    warning('The WindowButtonMotionFcn for the current figure ''%s'' has been unexpectedly executed.', get(f, 'Name'));%TO012009B
end

return;

%------------------------------------------------
function vertex = findRectangleVertex(hObject)

%Get the current location of the mouse.
currentPosition = get(gca, 'currentPoint');
currentPosition = currentPosition(1, 1:2);%Only consider the (X, Y, 0) plane.

pos = get(hObject, 'Position');

%Find the vertex with the smallest Euclidean distance from the mouseclick, that's the one to be moved.
x = sqrt((currentPosition(1) - [pos(1) pos(1)+pos(3)]).^2);
y = sqrt((currentPosition(2) - [pos(2) pos(2)+pos(4)]).^2);
verticesDistances = [x(1)+y(1) x(2)+y(1) x(1)+y(2) x(2)+y(2)];
[mn vertex] = min(verticesDistances);

return;