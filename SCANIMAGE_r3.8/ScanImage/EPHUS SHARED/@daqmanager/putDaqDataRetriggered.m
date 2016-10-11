% @daqmanager/putDaqDataRetriggered - Rebuffer new data, as part of the same acquisition.
%
% SYNTAX
%  putDaqDataRetriggered(dm, channelName, data)
%    dm - The @daqmanager instance.
%    channelName - The channel to put data out to. May be a cell array of channel names, if all channels are on the same board.
%    data - The data to be put out. May be a cell array, of the same length as the channelName cell array.
%
% USAGE
%
% NOTES
%  No properties are changed, no events are processed. A restart is executed, but no trigger should be issued.
%  Copy & paste job from stim_Start and ephys_Start (refactoring).
%
% CHANGES
%  TO101006A - Handle Matlab's idiotic cell/string nonsense properly. -- Tim O'Connor 10/10/06
%
% Created 8/28/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function putDaqDataRetriggered(this, channelName, data)

if strcmpi(class(channelName), 'char')
    channelName = {channelName};
    data = {data};
else
    if length(channelName) ~= length(data)
        error('Number of channel names, %s, does not match number of data arrays, %s.', ...
            num2str(length(channelName)), num2str(length(data)));
    end
end

ao = getAO(this, channelName{1});
if isempty(ao)
    error('No AO found for channel ''%s''.', channelName{1});
end
%TO101006A
activeChannelList = ao.Channel(:).ChannelName;
if strcmpi(class(activeChannelList), 'char')
    activeChannelList = {activeChannelList};
end
if ~all(ismember(channelName, activeChannelList))
    error('All channels specified in a single call to @daqmanager/putDaqDataRetriggered must be on the same board and running.');
end

% % fprintf(1, '\n');
% fprintf(1, '%s - @daqmanager/putDaqDataRetriggered: Stopping object...\n', datestr(now));
% % ao
% % get(ao)
% % fprintf(1, '---------------------------------------------------------\n');
startFcn = get(ao, 'StartFcn');
stopFcn = get(ao, 'StopFcn');
set(ao, 'StopFcn', '');
set(this, 'restartingChannelForChannelAddition', 1);
stop(ao);

% channelNameString = '';
% for i = 1 : length(channelName)
%     channelNameString = [channelNameString channelName{i} ', '];
% end
% fprintf(1, '@daqmanager/putDaqDataRetriggered: ''%s''\n%s', channelNameString, getStackTraceString);
% getAO(this, 'xMirror')
% fprintf(1, '-----------------------\n\n');

clearAOData(this, channelName{1});
% fprintf(1, '@daqmanager/putDaqDataRetriggered: Cleared object now has %s samples available\n', num2str(get(ao, 'SamplesAvailable')));
for i = 1 : length(channelName)
% fprintf(1, '@daqmanager/putDaqDataRetriggered: Putting %s samples on channel ''%s''\n', num2str(length(data{i})), channelName{i});
    putDaqData(this, channelName{i}, data{i});
end
% fprintf(1, '@daqmanager/putDaqDataRetriggered: Updating outputdata...\n');
updateOutputData(this, channelName);%TO081406A
% fprintf(1, '@daqmanager/putDaqDataRetriggered: Object now has %s samples available\n', num2str(get(ao, 'SamplesAvailable')));

% % % figure, plot(1:length(data{1}), data{1}, 'o:', 1:length(data{2}), data{2}, 's:');
% fprintf(1, '%s - @daqmanager/putDaqDataRetriggered: Restarting object...\n', datestr(now));
% % ao
% % get(ao)
% % fprintf(1, '---------------------------------------------------------\n');
% % fprintf(1, '---------------------------------------------------------\n\n\n');
set(ao, 'StartFcn', startFcn, 'StopFcn', stopFcn);
start(ao);
set(this, 'restartingChannelForChannelAddition', 0);

return