function updateChannelColormap(h)
%% function updateChannelColormap
% Function that executes whenever the user selects a different colormap
% (for a specific channel).
%   
global state gh

fieldValue = get(h,'String');

try
    map = eval(fieldValue);
catch ME
    error('You have entered an invalid expression.');
end

% ensure our call returned an Nx3 matrix
[~,dim] = size(map);

if dim ~= 3
   error('You have entered an invalid expression.');
end

% determine which channel's colormap is being updated
title = get(h,'Tag');
channel = str2double(title(end));

% update our state variable to store the new value
% escapedValue = strrep(fieldValue,'''','''''');
% escapedValue = ['$' escapedValue];
% eval(['state.internal.figureColormap' num2str(channel) ' = escapedValue;']);

try
    hFigure = state.internal.GraphFigure(channel);
    set(hFigure,'Colormap',map);
    refresh(hFigure);
catch e
   % do nothing 
end