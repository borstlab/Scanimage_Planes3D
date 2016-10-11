% @daqmanager/getDaqSample - Retrieve a sample from an analoginput.
%
% SYNTAX
%  sample = getDaqSample(this, name)
%   name - A valid channel name (or cell array of channel names).
%   sample - A single sample from the named channel (or vector of samples, if multiple names are supplied).
%
% USAGE
%
% NOTES
%  See TO012706F, TO080406E, and TO080606A for more details.
%  Copy & paste from @daqmanager/putDaqSample.
%
% CHANGES
%    TO080806A: Forgot to delete the existing channels. Changed conditions for restart. -- Tim O'Connor 8/8/06 
%    TO080806B: For now, remove all channels, see TODO_080606. -- Tim O'Connor 8/8/06
%
% Created 8/6/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function sample = getDaqSample(dm, name)
global gdm;

%TODO: For now, implement this the slow way, "vectorize" it (doing multiple channels on the same board simultaneously) later.
%      Currently, it's used rarely (only in @axopatch_200B/update), so it's not a priority.  (as of 8/6/06)
if strcmpi(class(name), 'cell')
    warning('Call to @daqmanager/putDaqSample using multiple channels simultaneously. This feature is not optimally implemented, and can be sped up significantly.');
    if isempty(name)
        error('Call to @daqmanager/putDaqSample using empty cell array when specifying channel name(s).');
    end
    sample = zeros(size(name));
    for i = 1 : length(name)
        sample(i) = getDaqSample(dm, name{i});
    end
    return;
end

index = getChannelIndex(dm, name);
if index < 1
    errMsg = sprintf('No channel found with name: ''%s''', name);
    error(errMsg);
end

ai = getAI(dm, name);

%TO080406E
channelList = {};
restartRequired = 0;
if ~isempty(ai)
    %TO013106D: Only restart those boards which have been stopped.
    %TO080806B: For now, remove all channels, see TODO_080606
    if ~isempty(ai.Channel)
        if strcmpi(get(ai, 'Running'), 'On')
            if get(ai, 'TriggersExecuted') == 0
                stopFcn = get(ai, 'StopFcn');
                startFcn = get(ai, 'StartFcn');
                gdm(dm.ptr).restartingChannelForChannelAddition = 1;
                stop(ai);
                gdm(dm.ptr).restartingChannelForChannelAddition = 0;
            else
                error('@daqmanager/getDaqSample: Channel ''%s'' is in use.', name);
            end
            restartRequired = 1;
        end
        channelList = ai.Channel.ChannelName;
        delete(ai.Channel);%TO080806A
    end
end

%Create the channel.
addInputChannel(dm, gdm(dm.ptr).channels(index).boardId, gdm(dm.ptr).channels(index).channelId, gdm(dm.ptr).channels(index).name);

%Read the single value.
sample = getsample(ai);

%Get rid of that channel.
% delete(ai.Channel(gdm(dm.ptr).channels(index).channelId));%TO08095E
delete(ai.Channel);

if ~isempty(channelList)
    createChannels(dm, channelList);
    setProperties(dm, channelList);
    clearBuffers(dm, channelList);
    if restartRequired
        set(ai, 'StartFcn', startFcn, 'StopFcn', stopFcn);
        gdm(dm.ptr).restartingChannelForChannelAddition = 1;
        start(ai);
        gdm(dm.ptr).restartingChannelForChannelAddition = 0;
    end
end

return;