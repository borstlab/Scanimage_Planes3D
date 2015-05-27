function expandAxes(handle,varargin)
% EXPANDAXES   - takes an axes handle and moves all graphics objects to different axes.
% 	EXPANDAXES will look on the current axes handle specified for children objects and set all
% 	the 'Parent' properties of those objects to different axes int he same
% 	figure. 
%
% 	EXPANDAXES(axhandle) will look on axhandle for children objects and set all
% 	the 'Parent' properties of those objects to different axes in the same
% 	figure. 
% 	
%   EXPANDAXES(axhandle,varagin) will look on axhandle for children objects and set all
% 	the 'Parent' properties of those objects to different axes in the same
% 	figure.  Varargin are param/value pairs that specify how to splay the
% 	new axes created.  The param is called 'tilestyle' and possible values
% 	are 'vertical, 'horizontal', or 'tile'.
%
% 	See also COLLAPSEAXES, SPLAYAXESTILE

% expands waves on axes to new axes in figure spalyed axes 
% varargin csn set the way to display the expanded axes: 
% 'tilestyle' can be: 'vertical, or 'horizontal'

if nargin < 1
    handle=gca;
elseif ~istype(handle,'axes')
    error('expandAxes: Must supply a figure handle');
end
f=get(handle,'Parent');
allAx=length(findobj(f,'type','axes'));
plotarray=[findobj(handle,'type','line') findobj(handle,'type','image')];
plotarray=unique(plotarray);

if isempty(plotarray)
    return
else
    for objCounter=2:length(plotarray)
        UserData.autoScale=1;
        ax=axes('Parent', f, 'Color', 'None', 'NextPlot', 'add', 'UserData', UserData, ...
            'XLimMode', 'auto', 'YLimMode', 'auto', 'Tag', ['Axis' num2str(allAx+1)]);
        setWaveContextMenu(ax);
        set(plotarray(objCounter),'Parent',ax);
        rescaleAxis(ax);
    end
end

tilestyle='Tile';
% Parse input parameter pairs and rewrite values.
counter=1;
while counter+1 <= length(varargin)
    eval([varargin{counter} '=''' (varargin{counter+1}) ''';']);
    counter=counter+2;
end
eval(['splayAxis' tilestyle '(f);']);




