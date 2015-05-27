% @daqmanager/updateOutputData
%
% SYNTAX
%  updateOutputData(dm, channelName)
%    dm - @daqmanager instance.
%    channelName - The name of the channel for which to put data to the hardware buffer (may be a cell array of names).
%
% USAGE
%
% NOTES
%  This is mostly a copy & paste from @daqmanager/putDaqSample.
%  See TO081406A.
%
% CHANGES
%  TO083006A - Vectorize (in the case of all channels being on the same board). -- Tim O'Connor 8/30/06
%  TO101006A - Handle Matlab's idiotic cell/string nonsense properly. -- Tim O'Connor 10/10/06
%
% Created 8/14/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function updateOutputData(dm, channelName)
global gdm;

%TO083006A - Handle data across a single board in one shot.
%TODO: For now, implement this the slow way, "vectorize" it (doing multiple channels on the same board simultaneously) later.
%      Currently, it's used rarely, and never for multiple channels at once, so it's not a priority.  (as of 8/6/06)
if strcmpi(class(channelName), 'cell')
    ao = getAO(dm, channelName{1});
    %TO101006A
    activeChannelList = ao.Channel(:).ChannelName;
    if strcmpi(class(activeChannelList), 'char')
        activeChannelList = {activeChannelList};
    end
    if ~all(ismember(channelName, activeChannelList))
        warning('Call to @daqmanager/updateOutputData updating multiple channels simultaneously. This feature is not optimally implemented, and can be sped up significantly.');
        for i = 1 : length(channelName)
            updateOutputData(dm, channelName{i});
        end
        return;
    end
else
    ao = getAO(dm, channelName);
    channelName = {channelName}
end

errMsg = '';
for i = 1 : length(channelName)
    index = getChannelIndex(dm, channelName{i});
    if index < 1
        errMsg = sprintf('%sNo channel found with name: ''%s''\n', errMsg, channelName{i});
    end
end
if ~isempty(errMsg)
    error(errMsg);
end

%TO080406E
channelList = {};
restartRequired = 0;
if ~isempty(ao)
    %TO013106D: Only restart those boards which have been stopped.
    %TO080806B: For now, remove all channels, see TODO_080606
    if ~isempty(ao.Channel)
        if strcmpi(get(ao, 'Running'), 'On')
            if get(ao, 'TriggersExecuted') == 0
                stopFcn = get(ao, 'StopFcn');
                startFcn = get(ao, 'StartFcn');
                gdm(dm.ptr).restartingChannelForChannelAddition = 1;
                stop(ao);
                gdm(dm.ptr).restartingChannelForChannelAddition = 0;
            else
                error('@daqmanager/updateOutputData: Channel ''%s'' is in use.', channelName{1});
            end
            restartRequired = 1;
        end
        channelList = ao.Channel.ChannelName;
%         delete(ao.Channel);%TO080806A
    end
end

if ~isempty(channelList)
%     createChannels(dm, channelList);
%     setProperties(dm, channelList);
    addData(dm, channelList);
    if restartRequired
        set(ao, 'StartFcn', startFcn, 'StopFcn', stopFcn);
        gdm(dm.ptr).restartingChannelForChannelAddition = 1;
        start(ao);
        gdm(dm.ptr).restartingChannelForChannelAddition = 0;
    end
end

return;