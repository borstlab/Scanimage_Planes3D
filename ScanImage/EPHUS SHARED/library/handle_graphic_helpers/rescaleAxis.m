function rescaleAxis(axishandle,varargin)
%RESCALEAXIS   - customized scaling of axes based on data on axis.
% RESCALEAXIS rescales the current axes to match the data better.
%
% RESCALEAXIS(axishandle) rescales the axishandle passed in to match the data better.
%
% RESCALEAXIS(axishandle,param,value) rescales the axishandle passed in to
% match the data better with the following options:
% 
% 	xfraction/yfraction:  to specify range of X/YLims.
% 	scalex/scaley:  to 0 (don't scale that direction) or 1 (scale that direction).
% 	reorder:        equal 0 (don't reorder axis children) or 1 to reorder the children.
%
%   See also RESHUFFLEAXISHANDLES

if nargin == 0
    a=findobj('Type', 'axes');  % Are there any axes?
    if ~isempty(a)              
        ax=gca;
    else
        return
    end   
else
    if mod(length(varargin),2)==1	% passed an axishandle...
        varargin=[{axishandle} varargin];
        a=findobj('Type', 'axes');  % Are there any axes?
        if ~isempty(a)              
            ax=gca;
        else
            return
        end   
    elseif mod(length(varargin),2)==0	
        if istype(axishandle,'axes')
            ax=axishandle;
        else      
            error('rescaleAxis: invalid axes handle.');
        end
    end
end

autoScale=1;
userData = get(ax, 'UserData'); % Check if frozen....the dont autoscale.
if isfield(userData, 'autoScale')
    autoScale = userData.autoScale;
else
    return
end

xfraction=0;
yfraction=.05;
scalex=1;
scaley=1;
reorder=0;

% Parse input parameter pairs and rewrite values.
counter=1;
while counter+1 <= length(varargin)
    eval([varargin{counter} '=' num2str(varargin{counter+1}) ';']);
    counter=counter+2;
end

if autoScale==0
    set(ax,'XLimMode','manual','YLimMode','manual');
    return
elseif autoScale==1 % Wave class autoscale.....
    set(ax,'XLimMode','manual','YLimMode','manual');
elseif autoScale==2	% Matlab autoscale.....
	if reorder
        reshuffleAxisHandles(ax);
	end
	return
end

allXData=[];
allYData=[];

plots=[findobj(ax, 'Type', 'line') findobj(ax, 'Type', 'image')]; % Are there any lines or images? 
XData = get(plots, 'XData');		% Get the XData of all the plots
YData = get(plots, 'YData');		% Get the XData of all the plots

if ~iscell(XData)
    XData={XData};
end

if ~iscell(YData)
    YData={YData};
end

for i = 1:length(XData)
    allXData=[allXData XData{i}];
    allYData=[allYData YData{i}];
end

allXData=allXData(~isnan(allXData) & ~isinf(allXData));
allYData=allYData(~isnan(allYData) & ~isinf(allYData));

setAxesProps=1;
% if isempty(allXData) | isempty(allYData)	% BSMOD comment out
%     setAxesProps=0;
% end

if setAxesProps
    minX = min(allXData);
    maxX = max(allXData);
    minY = min(allYData);
    maxY = max(allYData);
    
    if scaley
        if minY > maxY 
            set(ax, 'YLim', [0 1]);
		elseif isempty(minY) | isempty(maxY)
            set(ax, 'YLim', [0 1]);
        elseif minY==maxY
            set(ax, 'YLim', [minY-1e-3 minY+1e-3]);
        else
            YRange=maxY-minY;
            set(ax, 'YLim', [minY-yfraction*(YRange) maxY+yfraction*(YRange)]);
        end
    end
    
    if scalex
        if minX > maxX 
            set(ax, 'XLim', [0 1]);
		elseif isempty(minX) | isempty(maxX)
            set(ax, 'XLim', [0 1]);
        elseif minX==maxX
            set(ax, 'XLim', [minX-1e-3 minX+1e-3]);
        else			
            XRange=maxX-minX;
            set(ax, 'XLim', [minX-xfraction*(XRange) maxX+xfraction*(XRange)]);
        end
    end
end

if reorder
    reshuffleAxisHandles(ax);
end

