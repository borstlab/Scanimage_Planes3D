% drawPolygon
%
% SYNTAX
%  points = drawPolygon
%  points = drawPolygon(ax)
%  points = drawPolygon(PROPERTY_NAME, PROPERTY_VALUE, ...)
%  [points lineHandle] = drawPolygon
%  [points lineHandle] = drawPolygon(ax)
%  [points lineHandle] = drawPolygon(PROPERTY_NAME, PROPERTY_VALUE, ...)
%   points - A 2xN array of X-Y values.
%   ax - A handle to an axes.
%   lineHandle - The handle to the line object, drawn while creating the polygon.
%   PROPERTY_NAME - Any valid property name string.
%   PROPERTY_VALUE - The value associated with the preceding propery value.
%    Multiple PROPERTY_NAME-PROPERTY_VALUE pairs are allowed.
%
% PROPERTIES
%  Axes - The axes on which to draw a closed polygon. Default: `gca`
%  Color - An RGB color triplet, specifying the color of the line. Default: [0 0.5 0]
%  LineStyle - The style of the line to be drawn. Default: '-'
%  LineWidth - The width of the line to be drawn, in points. Default: 0.5
%  Marker - The marker to be used on line at positions where the mouse was clicked. Default: 'None'
%  MarkerSize - The size of markers, in points. Default: 6
%
% USAGE
%  This function will return a set of points, defining a closed polygon, on an axes.
%  The user must click the points with a mouse, terminating the input with a double-click or a right-click.
%
% NOTES
%
% CHANGES
%   VI071310A: Use getRectFromAxes()/getPointsFromAxes for selection of rectangular area & points, respectively -- Vijay Iyer 7/13/10
%
% SEEALSO
%  line
%  getline
%  makeGraphicsObjectMutable
%
% Created 8/16/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function varargout = drawPolygon(varargin)

%Defaults.
ax = [];%Handle `gca` later, so it doesn't spawn an erroneous figure and/or axes.
color = [0 0.5 0];
lineStyle = '-';
lineWidth = 0.5;
marker = 'None';
markerSize = 6;

%Simplified form, taking just an axes argument.
if length(varargin) == 1
    if isnumeric(varargin{1}) & ishandle(varargin{1})
        ax = varargin{1};
    else
        error('Invalid axes specified.');
    end
end
if length(varargin) > 1 & mod(length(varargin), 2) ~= 0
    error('Wrong number of arguments. Key-value pairs must be specified.');
end

%Process args.
if length(varargin) > 1
    for i = 1 : length(varargin)
        switch lower(varargin{i})
            case 'axes'
                ax = varargin{i + 1};
            case 'color'
                color = varargin{i + 1};
                if ~isnumeric(color) & length(color) == 3
                    error('Bad color specification. Must be an RGB triplet.');
                end
            case 'linestyle'
                lineStyle = varargin{i + 1};
            case 'linewidth'
                lineWidth = varargin{i + 1};
            case 'marker'
                marker = varargin{i + 1};
            case 'markersize'
                markerSize = varargin{i + 1};
            otherwise
                error('Unrecognized property name: %s', varargin{i});
        end
    end
end

%Default to `gca`.
if isempty(ax)
    ax = gca;
end

%Get the points that define the perimeter of the polygon.
[x y] = getPointsFromAxes(ax,'numberOfPoints',2,'nomovegui',1); %VI071310A

%Close the polygon.
if x(1) ~= x(end) | y(1) ~= y(end)
    x(length(x) + 1) = x(1);
    y(length(y) + 1) = y(1);
end

%Draw the line.
lineHandle = line('Parent', ax, 'XData', x, 'YData', y, 'Color', color, 'LineStyle', lineStyle, 'LineWidth', lineWidth, ...
    'Marker', marker, 'MarkerSize', markerSize, 'MarkerEdgeColor', color, 'MarkerFaceColor', color);

%Marshal the outputs.
varargout{1} = cat(2, x, y);
if nargout == 2
    varargout{2} = lineHandle;
end    

return;