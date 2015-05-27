%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Call through to putsample.
%%
%%  OBJ = putDaqSample(OBJ, name, sample)
%%    name - Channel name (or cell array of channel names).
%%    sample - Sample to be put out (or array of samples).
%%
%%  Created - Tim O'Connor 11/24/03
%%
%%  Changed:
%%    1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%    8/4/06 Tim O'Connor TO080406E: Implement "multiStart" functionality. See TO012706F (@daqmanager/startChannel) for more details.
%%    8/6/06 Tim O'Connor TO080606A: Continued TO080406E after being interrupted by a massive power failure.
%%    8/8/06 Tim O'Connor TO080806A: Forgot to delete the existing channels. Changed conditions for restart.
%%    8/8/06 Tim O'Connor TO080806B: For now, remove all channels, see TODO_080606.
%%    8/15/06 Tim O'Connor TO081506A: Complete rewrite to deal with the following inconsistency that appears in the daq engine -
%%                                    Display Summary of Analog Output (AO) Object Using 'PCI-6713'.
%%                                           Output Parameters:  10000 samples per second on each channel.                 <-- HOW IS THIS POSSIBLE?!? :: 4000/10000 ~= 0.8
%%                                          Trigger Parameters:  1 'HwDigital' trigger.
%%                                               Engine status:  Waiting for trigger.
%%                                                               0.8 total sec. of data currently queued per trigger.      <-- HOW IS THIS POSSIBLE?!? :: 4000/10000 ~= 0.8
%%                                                               4000 samples currently queued by PUTDATA.                 <-- HOW IS THIS POSSIBLE?!? :: 4000/10000 ~= 0.8
%%                                                               0 samples sent to output device since START.
%%                                    AO object contains channel(s):
%%                                       Index:  ChannelName:   HwChannel:  OutputRange:  UnitsRange:  Units:   
%%                                       1       '700A-2-VCom'  3           [-10 10]      [-10 10]     'Volts'  
%%    8/30/06 Tim O'Connor TO083006B: Re-put data, if the channels have been stopped. -- Tim O'Connor.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dm = putDaqSample(dm, name, sample)
global gdm;

if strcmpi(class(name), 'char')
    name = {name};
    if length(name) ~= length(sample)
        error('Call to @daqmanager/putDaqSample using multiple channels simultaneously. Named %s channels, but supplied %s samples.', num2str(length(name)), num2str(length(sample)));
    end
end

channelIndices = zeros(size(name));
boardIDs = zeros(size(name));
for i = 1 : length(name)
    channelIndices(i) = getChannelIndex(dm, name{i});
    boardIDs(i) = gdm(dm.ptr).channels(channelIndices(i)).boardId;
    channelIDs(i) = gdm(dm.ptr).channels(channelIndices(i)).channelId;
end

[boards, firstOccurrences, reverseMapping] = unique(boardIDs);
for i = 1 : length(boards)
    ao = gdm(dm.ptr).aos{boards(i)};
% fprintf(1, '@daqmanager/putDaqSample: Board %s - ''%s''\n', num2str(boards(i)), get(ao, 'Tag'));
    realAO = [];
    restartRequired = 0;
    deleteChannels = 0;
    [groupedChannels groupingOrder] = sort(channelIDs(find(boards(i) == boardIDs)));
    groupedSamples = sample(find(boards(i) == boardIDs));
    groupedSamples = groupedSamples(groupingOrder);

    %Make sure the analogoutput object is ready, with all channels created.
    if ~isempty(ao.Channel)
        existingChannels = ao.Channel.HwChannel;
        if strcmpi(class(existingChannels), 'cell')
            existingChannels = [existingChannels{:}];
        end
        
        %See if the device is in use, and if a restart is allowed.
        if strcmpi(get(ao, 'Running'), 'On') & gdm(dm.ptr).allowMultistart
            if get(ao, 'TriggersExecuted') == 0
% fprintf(1, '@daqmanager/putDaqSample: Stopping analogoutput...\n');
                stopFcn = get(ao, 'StopFcn');
                startFcn = get(ao, 'StartFcn');
                gdm(dm.ptr).restartingChannelForChannelAddition = 1;
                stop(ao);
                gdm(dm.ptr).restartingChannelForChannelAddition = 0;
            else
                error('@daqmanager/putDaqSample: Channel ''%s'' is in use, a trigger has been executed.\n%s', name{firstOccurrences}, getStackTraceString);
            end
            restartRequired = 1;
        elseif ~gdm(dm.ptr).allowMultistart
            error('@daqmanager/putDaqSample: Channel ''%s'' is in use, ''multistart'' functionality is not allowed.\n%s', name{firstOccurrences}, getStackTraceString);
        end
        
        %Make sure the channels are properly configured.
        if ~all(ismember(existingChannels, groupedChannels))
% fprintf(1, '@daqmanager/putDaqSample: Creating temporary analogoutput...\n');
            realAO = ao;
            if gdm(dm.ptr).nidaqmxEnabled
                ao = analogoutput(gdm(dm.ptr).adaptor, ['Dev' num2str(boards(i))]);
            else
                ao = analogoutput(gdm(dm.ptr).adaptor, boards(i));
            end
% fprintf(1, '@daqmanager/putDaqSample: Creating temporary channels on a temporary analogoutput...\n');
            for j = 1 : length(groupedChannels)
                addchannel(ao, groupedChannels(j), ['putDaqSampleTemp_' num2str(groupedChannels(j))]);
            end
        end
    else
% fprintf(1, '@daqmanager/putDaqSample: Creating temporary channels on the real analogoutput...\n');
        for j = 1 : length(groupedChannels)
            deleteChannels = 1;
            addchannel(ao, groupedChannels(j), ['putDaqSampleTemp_' num2str(groupedChannels(j))]);
        end
    end
% fprintf(1, '@daqmanager/putDaqSample: Putting sample(s)...\n');
    putsample(ao, groupedSamples);
    
    if deleteChannels
% fprintf(1, '@daqmanager/putDaqSample: Deleting temporary channels...\n');
        delete(ao.Channel);
        ao = realAO;
    end

    if ~isempty(realAO)
% fprintf(1, '@daqmanager/putDaqSample: Deleting temporary analogoutput...\n');
        delete(ao);
        ao = realAO;
    end
    
    if restartRequired
% fprintf(1, '@daqmanager/putDaqSample: Restarting analogoutput...\n');
        set(ao, 'StartFcn', startFcn, 'StopFcn', stopFcn);
        gdm(dm.ptr).restartingChannelForChannelAddition = 1;
        if get(ao, 'SamplesAvailable') == 0
            addData(dm, ao.Channel(:).ChannelName);%TO083006B
        end
        start(ao);
        gdm(dm.ptr).restartingChannelForChannelAddition = 0;
    end
end

return;
%TO081506A - Complete rewrite...
% %TODO: For now, implement this the slow way, "vectorize" it (doing multiple channels on the same board simultaneously) later.
% %      Currently, it's used rarely, and never for multiple channels at once, so it's not a priority.  (as of 8/6/06)
% if strcmpi(class(name), 'cell')
%     warning('Call to @daqmanager/putDaqSample using multiple channels simultaneously. This feature is not optimally implemented, and can be sped up significantly.');
%     if length(name) ~= length(sample)
%         error('Call to @daqmanager/putDaqSample using multiple channels simultaneously named %s channels, but supplied %s samples.', num2str(length(name)), num2str(length(sample)));
%     end
%     for i = 1 : length(name)
%         putDaqSample(dm, name{i}, sample(i));
%     end
%     return;
% end
% 
% index = getChannelIndex(dm, name);
% if index < 1
%     errMsg = sprintf('No channel found with name: ''%s''', name);
%     error(errMsg);
% end
% 
% ao = getAO(dm, name);
% 
% %TO080406E
% channelList = {};
% restartRequired = 0;
% if ~isempty(ao)
%     %TO013106D: Only restart those boards which have been stopped.
%     %TO080806B: For now, remove all channels, see TODO_080606
%     if ~isempty(ao.Channel)
%         if strcmpi(get(ao, 'Running'), 'On')
%             if get(ao, 'TriggersExecuted') == 0
%                 stopFcn = get(ao, 'StopFcn');
%                 startFcn = get(ao, 'StartFcn');
%                 gdm(dm.ptr).restartingChannelForChannelAddition = 1;
%                 stop(ao);
%                 gdm(dm.ptr).restartingChannelForChannelAddition = 0;
%             else
%                 error('@daqmanager/putDaqSample: Channel ''%s'' is in use.', name);
%             end
%             restartRequired = 1;
%         end
%         channelList = ao.Channel.ChannelName;
%         delete(ao.Channel);%TO080806A
%     end
% end
% 
% %Create the channel.
% addOutputChannel(dm, gdm(dm.ptr).channels(index).boardId, gdm(dm.ptr).channels(index).channelId, gdm(dm.ptr).channels(index).name);
% 
% %Output the single value.
% putsample(ao, sample);
% 
% %Get rid of that channel.
% % delete(ao.Channel(gdm(dm.ptr).channels(index).channelId));%TO08095E
% delete(ao.Channel);
% 
% if ~isempty(channelList)
%     createChannels(dm, channelList);
%     setProperties(dm, channelList);
%     addData(dm, channelList);
%     if restartRequired
%         set(ao, 'StartFcn', startFcn, 'StopFcn', stopFcn);
%         gdm(dm.ptr).restartingChannelForChannelAddition = 1;
%         start(ao);
%         gdm(dm.ptr).restartingChannelForChannelAddition = 0;
%     end
% end
% 
% return;