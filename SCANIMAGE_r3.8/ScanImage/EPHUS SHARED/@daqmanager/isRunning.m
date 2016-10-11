%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  See if a channel is running.
%%
%%  Returns 0 if the channel is not running, a number
%%  greater than 0 otherwise.
%%
%%  logical = isRunning(OBJ, name)
%%  logical = isRunning(OBJ, boardId, channelId, ioFlag)
%%   ioFlag - 1 if the channel is an input channel, 0 if it's an output channel.
%%
%%  Created - Tim O'Connor 6/24/05
%%
%%  Changed:
%%   TO091106B - Finally implemented this function. -- Tim O'Connor 9/11/06
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2005
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function yOrN = isRunning(dm, varargin)

if length(varargin) == 1
    ao = getAO(dm, varargin{1});
elseif length(varargin) == 3
    ao = getAO(dm, getChannelName(dm, varargin{:}));
else
    error('@daqmanager/isRunning: Illegal number of input arguments.');
end

yOrN = 0;
if strcmpi(get(ao, 'Running'), 'On')
    if length(varargin) == 1
        if ~isempty(find(strcmp(varargin{1}, ao.Channel.ChannelName)))
            yOrN = 1;
        end
    elseif length(varargin) == 3
        if any([ao.Channel.HwChannel] == varargin{2})
            yOrN = 1;
        end
    
    end
end

return;