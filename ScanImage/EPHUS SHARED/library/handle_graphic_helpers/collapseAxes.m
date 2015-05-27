function collapseAxes(handle)
% COLLAPSEAXES   - takes multiple axes on a figure, and moves all graphics objects to one axes.
% 	COLLAPSEAXES will look on the current figure handle specified for axes and set all
% 	the 'Parent' properties of those objects to the same axes.  It will then remove all
% 	the axes that contain no objects, and resize the remaining axes to fit the window.
%
% 	COLLAPSEAXES(fighandle) will look on the figure handle specified for axes and set all
% 	the 'Parent' properties of those objects to the same axes.  It will then remove all
% 	the axes that contain no objects, and resize the remaining axes to fit the window.
% 	
% 	See also EXPANDAXES, SPLAYAXESTILE

if nargin < 1
    handle=gcf;
elseif ~istype(handle,'figure')
    error('collapseAxes: Must supply a figure handle');
end

%Find axes to collapse to...
ax=findobj(handle,'type','axes');
if isempty(ax) | length(ax)==1
    return
end
mainAx=[];
tags=get(ax,'tag');
if any(strcmp(tags,'Axis1'))
    mainAx=ax(strcmp(tags,'Axis1'));
    ax(strcmp(tags,'Axis1'))=[];
else
    mainAx=ax(1);
    ax(1)=[];
end
     
for axiscounter=1:length(ax)
    wvs=[findobj(handle,'type','line') findobj(handle,'type','image')];
    if isempty(wvs)
        delete(ax(axiscounter));
        continue
    else
        for plotCounter=1:length(wvs)
            set(wvs(plotCounter),'Parent',mainAx);
        end
        delete(ax(axiscounter));
    end
end
rescaleAxis(mainAx);
splayAxisHorizontal(handle);



