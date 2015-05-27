% DRAWBOX
%
% Adds a user selected rectangle to a figure, and implements the ability to
% move and resize the rectangle (via the ButtonDownFcn callback).
%
% drawBox - Operates on the current figure, as determined by gcf.
% drawBox(f) - Takes a figure to draw a box on.
% drawBox(PropertyName, PropertyValue, ...) - Sets initial properties for the box.
% drawBox(f, PropertyName, PropertyValue, ...) - Sets initial properties for the box.
%
% For the caller to know that the box has been updated, they must either check the
% rectangle's Position property and/or implement a ButtonDownFcn (the resize/move will
% take place before the user's callback is executed).
%
% See Also GETRECT, RECTANGLE, RBBOX
%
%% CHANGES
%   VI071310A: Use getRectFromAxes() for selection of rectangular area -- Vijay Iyer 7/13/10
%
%% CREDITS
% Created: Tim O'Connor 5/28/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
%% *********************************************
function rect = drawBox(varargin)

vararginIndex = 1;
if isempty(varargin)
    f = gcf;
elseif ~ishandle(varargin{1})
    f = gcf;
else
    f = varargin{1};
    vararginIndex = 2;
end

%Shuffle things around and stuff any ButtonDownFcn into the user data, leave everything else,
%including the user data, intact.
udata = [];
userButtonDownFcn = '';
udataIndex = 0;
buttonDownIndex = 0;
for i = 1 : 2 : length(varargin)
    if strcmpi(varargin{i}, 'ButtonDownFcn')
        buttonDownIndex = i + 1;
        userButtonDownFcn = varargin{buttonDownIndex};
    elseif strcmpi(varargin{i}, 'UserData')
        udataIndex = i + 1;
        udata = varargin{udataIndex};
    end
end
udata.userButtonDownFcn = userButtonDownFcn;

if udataIndex
    varargin{udataIndex} = udata;
else
    varargin{length(varargin) + 1} = 'UserData';
    varargin{length(varargin) + 1} = udata;
end

if buttonDownIndex
    varargin{buttonDownIndex} = @btnDnFcn;
else
    varargin{length(varargin) + 1} = 'ButtonDownFcn';
    varargin{length(varargin) + 1} = @btnDnFcn;
end

%Select a rectangle.
pos = getRectFromAxes(get(f,'CurrentAxes'),'Cursor','crosshair','nomovegui',1); %VI071310A
pos
%Draw a rectangle, and return the handle.
rect = rectangle(varargin{vararginIndex : end}, 'Position', pos);

return;

%---------------------------------------------------------------------------------------------------
function btnDnFcn(varargin)

stretch = 1;
if strcmpi(get(gcf, 'SelectionType'), 'Normal')
    %Shift.
    stretch = 0;
end

%We want to work in pixels.
originalAxesUnits = get(gca, 'Units');
set(gca, 'Units', 'pixels');
%Get the position of the rectangle, axes, and figure.
oPos = get(varargin{1}, 'Position')
axPos = get(gca, 'Position')
fPos = get(gcf, 'Position')

try
    %Compute the scaling between axis coordinates (normalized) and pixels.
    xRatio = fPos(3) / axPos(3)
    yRatio = fPos(4) / axPos(4)
    xExtent = (axPos(3) - axPos(1))
    yExtent = (axPos(4) - axPos(2))
    normalizedOPos([1 3]) = oPos([1 3]) ./ diff(get(gca, 'XLim'));
    normalizedOPos([2 4]) = oPos([2 4]) ./ diff(get(gca, 'YLim'));
normalizedOPos

BOUND = rbbox;
% Get the boundary of the box
P = get(gca, 'Position');
XLimit = get(gca, 'Xlim');
YLimit = get(gca, 'Ylim');
% Get the coordinates of the lower left corner of the box
% Its height and width, and the X-Y coordinates of the 4 corners
DeltaX = XLimit(2)-XLimit(1);
DeltaY = YLimit(2)-YLimit(1);
LeftDist = BOUND(1)-P(1);
UpDist = BOUND(2)-P(2);
leftDist = oPos(1) + axPos(1);
upDist = oPos(2) + axPos(2);
% Defining some useful quantities which will be used often
x = XLimit(1) + DeltaX * leftDist / oPos(3);
width = DeltaX * (leftDist - axPos(3)) / oPos(3);
y = YLimit(1) + DeltaY * upDist / P(4);
height = DeltaY * (upDist - axPos(4)) / oPos(4);

    %The initial position must be defined in figure coordinates.
%     initialPosition = [fPos(1) (fPos(2) - (oPos(4) * yRatio)) (oPos(3)  * xRatio) (oPos(4)  * yRatio)]
    initialPosition = [(normalizedOPos(1) * xExtent + axPos(1)) (normalizedOPos(2) * yExtent + axPos(2) + normalizedOPos(4) * yExtent) ...
            (normalizedOPos(3) * xExtent) (normalizedOPos(4) * yExtent)]
% initialPosition = [x y width height]
%     initialPosition = [(oPos(1) * xExtent + axPos(1)) (oPos(2) * yExtent + axPos(2) + oPos(4) * yExtent) ...
%             (oPos(3) * xExtent) (oPos(4) * yExtent)]
% initialPosition = [212 220 59 66]
%     dragrect([figpoint(1) figpoint(2)-ymag*pos(4) xmag*pos(3) ymag*pos(4)]);
[212 220 59 66]
    cPos = get(gca, 'CurrentPoint');
% global cHandle;
% cHandle = rectangle('Position', initialPosition, 'Curvature', [1 1], 'EdgeColor', [1 0 0], 'FaceColor', [1 0 0])
    
    if stretch
        'Stretching...'
        newPos = rbbox(initialPosition)
    else
        'Moving...'
        newPos = dragrect(initialPosition)
    end

    return;
    set(varargin{1}, 'Position', newPos);
    
    try
        %Call the user's ButtonDownFcn callback.
        udata = get(varargin{1}, 'UserData');
        if structFieldExists('udata.userButtonDownFcn')
            if strcmpi(class(udata.userButtonDownFcn), 'char')
                eval(udata.userButtonDownFcn);
            else
                feval(udata.userButtonDownFcn, varargin{:});
            end
        end
    catch
        if strcmpi(class(udata.userButtonDownFcn), 'char')
            funcString = udata.userButtonDownFcn;
        else
            funcString = func2str(udata.userButtonDownFcn);
        end
        warning('Failed to execute supplied ButtonDownFcn (''%s'') during box move/stretch: %s', funcString, lasterr);
    end
catch
    warning('Error moving/stretching box ''%s'': %s', get(varargin{1}, 'Tag'), lasterr);
end

%Put the original units back.
set(gca, 'Units', originalAxesUnits);

return;