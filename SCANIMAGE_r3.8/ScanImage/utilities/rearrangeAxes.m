function rearrangeAxes(axishandle)
% This function rearranges the order of pltos and imaegs on an axis
% so that the image is displayed on the bottom...that wy the pltos can be seen.

if nargin == 0	% No inputs...look for GCA
    a=findobj('Type', 'axes');  % Are there any axes?
    if ~isempty(a)              
        ax=gca;
    else
        error('rearrangeAxes: No axes to shuffle');
    end
elseif nargin==1	% axis handle...check
    if all(ishandle(axishandle))
        ax=axishandle;
    else      
        error('rearrangeAxes: invalid axes handle.');
    end
else
    error('rearrangeAxes: too many inputs')
end

for j=1:length(ax)
    
    a = get(ax(j), 'Children');
    type = get(a, 'type');
    
    top=[];
    bottom=[];
    neither=[];
    
    if iscell(type)
        for i = 1:length(type)
            if strcmp(type{i}, 'image')
                bottom = [bottom; a(i)];    % make array of handles for the bottom
            elseif strcmp(type{i}, 'rectangle')  
                top = [top ; a(i)];     % lines go on top...
            else
                neither = [neither; a(i)];
            end
        end
    else
        top=a;  % only one hter....
    end
    set(ax(j), 'Children', [top;neither;bottom]);
end