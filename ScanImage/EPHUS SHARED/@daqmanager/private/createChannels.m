% @daqmanager/createChannels - Create a set of channels on the correct analoginput/analogoutput objects.
%
% SYNTAX
%  createChannels(this, name, ...)
%  createChannels(this, nameArray)
%   name - A valid channel name.
%   nameArray - A cell array of valid channel names.
%
% USAGE
%  Create channels from a list of channel names (the list may be a cell array).
%
% NOTES
%  Moved from a subfunction in startChannel to a private function in the flass.
%
% CHANGES
%  TO080406E: See @daqmanager/startChannel for changes prior to refactoring. -- Tim O'Connor 8/4/06
%  TO080906A: When samples exist in a board, and a channel is deleted, an unnecessary warning is printed. -- Tim O'Connor 8/9/06
%  VI050508A: Don't set the default voltage range to -10/10V--this causes incorrect voltage on channel start prior to triggering (apparent DAQ Toolbox bug in R2007b) -- Vijay Iyer 5/5/08
%
% Created 8/4/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function createChannels(dm, varargin)
global gdm;

if strcmpi(class(varargin{1}), 'cell') & length(varargin) == 1
    varargin = varargin{1};
end

% channelStringList = '';
% for i = 1 : length(varargin)
%     channelStringList = [channelStringList ', ' varargin{i}];
% end
% if ~isempty(channelStringList)
%     fprintf(1, '@daqmanager/private/createChannels - %s\n%s\n', channelStringList(3:end), getStackTraceString);
% end

indices = [];
channelIds = [];
for i = 1 : length(varargin)
    index = getChannelIndex(dm, varargin{i});
    %TO112205A - First organize the channels to be added, then do them in numeric order.
    if index > 0
        % There's a Matlab bug here, it does not like the gdm(dm.ptr).channels(index).channelId variable,
        % any other valid integer works (including the value of the variable), but reassigning the variable
        % to a different name results in the same error. Math can be performed on the variable without a problem.
        %
        % Here's the error message:
        %   NI-DAQ: You must provide a single buffer of interleaved data, and the channels
        %   must be in ascending order.  You cannot use DMA to transfer data from two
        %   buffers; however, you may be able to use interrupts.
        %
        % Setting the transfer mode to 'Interrupt' seems to help.
        %
        % Mathworks contact: snikumbh@mathworks.com (EDG/Tech Support)
        %                    rob.purser@mathworks.com (DAQ Toolkit Lead)
        % Case ID: 1346844
        indices(length(indices) + 1) = index;
        %channelIds(length(channelIds) + 1) = gdm(dm.ptr).channels(index).channelId;
        %if gdm(dm.ptr).channels(index).ioFlag == 0
        %    addOutputChannel(dm, gdm(dm.ptr).channels(index).boardId, gdm(dm.ptr).channels(index).channelId, varargin{i});
        %else
        %     addInputChannel(dm, gdm(dm.ptr).channels(index).boardId, gdm(dm.ptr).channels(index).channelId, varargin{i});
        %end
    else
        errMsg = sprintf('No channel found with name: ''%s''', varargin{i});
        error(errMsg);
    end
end

%TO112205A - First organize the channels to be added, then do them in numeric order.
[sorted order] = sort(indices);

%TO010606E - The addOutputChannel and addInputChannel functions were slow when called independently, a batched version
%            could be just as secure while running significantly faster. For now, code it right here (since there's no
%            other place that ever creates channels), with the option to break it out separately later.
indices = indices(order);

for i = 1 : length(indices)
    if gdm(dm.ptr).channels(indices(i)).ioFlag == 0
        ao = gdm(dm.ptr).aos{gdm(dm.ptr).channels(indices(i)).boardId};

        %TO012706F - Any existing channels need to get restarted for allowMultistart. -- Tim O'Connor 1/27/06
        if strcmpi(get(ao, 'Running'), 'On') & gdm(dm.ptr).allowMultistart
            %TO030606A - The stopFcn must not be executed on a channel restart while adding new channels. -- Tim O'Connor 3/6/06
            gdm(dm.ptr).restartingChannelForChannelAddition = 1;
            stop(ao);
            gdm(dm.ptr).restartingChannelForChannelAddition = 0;
        end
        if ~isempty(ao.Channel)
% fprintf(1, 'Adding output channel %s:%s:''%s''\n', num2str(gdm(dm.ptr).channels(indices(i)).boardId), ...
%     num2str(gdm(dm.ptr).channels(indices(i)).channelId), gdm(dm.ptr).channels(indices(i)).name);
            %TO080906A - When samples exist in a board, and a channel is deleted, an unnecessary warning is printed - "Warning: All data must be put again.".
            %            I have not figured out a good way to suppress this. -- Tim O'Connor 8/9/06
            %if get(ao, 'SamplesAvailable') > 0
            %    for j = 1 : length(ao.Channel)
            %        resetID(j) = ao.Channel(j).HwChannel;
            %        resetName{j} = ao.Channel(j).ChannelName;
            %        resetIndex(j) = ao.Channel(j).Index;
            %        resetOutputRange{j} = ao.Channel(j).OutputRange;
            %        resetUnitsRange{j} = ao.Channel(j).UnitsRange;
            %        resetUnits{j} = ao.Channel(j).Units;
            %    end
% fprintf(1, '@daqmanager/private/createChannels - Deleting existing channel(s)...\n');
            %    delete(ao.Channel);
            %    [resetIndexSorted resetSort] = sort(resetIndex);
            %    for j = 1 : length(resetSort)
% fprintf(1, '@daqmanager/private/createChannels - Re-adding existing channel ''%s''...\n%s', resetName{resetSort(j)}, getStackTraceString);
            %        addchannel(ao, resetID(resetSort(j)), resetIndex(resetSort(j)), resetName{resetSort(j)});
            %        set(ao.Channel(resetSort(j)), 'OutputRange', resetOutputRange{resetSort(j)}, 'UnitsRange', resetUnitsRange{resetSort(j)}, 'Units', resetUnits{resetSort(j)});
            %    end
            %end

            %TO014106A - Must convert into a numeric array in two steps, it can't be done in one. -- Tim O'Connor 1/31/06
            indexes = ao.Channel(:).Index;
            if isempty(indexes)
                indexes = 0;
            elseif length(indexes) > 1
                indexes = [indexes{:}];
            end
            %TO013106B: Make sure the same channel isn't added multiple times. -- Tim O'Connor 1/31/06
            if ismember(gdm(dm.ptr).channels(indices(i)).name, ao.Channel(:).ChannelName)
                error('Channel ''%s'' is already added to the board.', gdm(dm.ptr).channels(indices(i)).name);
            end
% fprintf(1, '@daqmanager/private/createChannels - Adding new channel ''%s''...\n', gdm(dm.ptr).channels(indices(i)).name);
            chan = addchannel(ao, gdm(dm.ptr).channels(indices(i)).channelId, ...
                min(max(indexes + 1), gdm(dm.ptr).channels(indices(i)).channelId + 1), gdm(dm.ptr).channels(indices(i)).name);%TO012005b
% fprintf(1, '@daqmanager/private/createChannels - Added new channel ''%s''.\n%s', gdm(dm.ptr).channels(indices(i)).name, getStackTraceString);
    
    %%%%COMMENTED OUT (VI050508A)
    %set(chan, 'OutputRange', [-10 10], 'UnitsRange', [-10 10]);%TO070606B - Enforce a default -10 to 10 V range. -- Tim O'Connor 7/6/06
    %%%%END COMMENTS
    
% fprintf(1, '@daqmanager/private/createChannels - Set OutputRange and UnitsRange to [-10, 10]\n\n');
% fprintf(1, 'Added output channel %s:%s:''%s''\n', num2str(gdm(dm.ptr).channels(indices(i)).boardId), ...
%     num2str(gdm(dm.ptr).channels(indices(i)).channelId), gdm(dm.ptr).channels(indices(i)).name);
        else
% fprintf(1, 'Adding output channel %s:%s:''%s''\n', num2str(gdm(dm.ptr).channels(indices(i)).boardId), ...
%     num2str(gdm(dm.ptr).channels(indices(i)).channelId), gdm(dm.ptr).channels(indices(i)).name);
% fprintf(1, '@daqmanager/private/createChannels - Adding new channel ''%s'' to empty board...\n', gdm(dm.ptr).channels(indices(i)).name);
            chan = addchannel(ao, gdm(dm.ptr).channels(indices(i)).channelId, 1, gdm(dm.ptr).channels(indices(i)).name);%TO012005b
% fprintf(1, '@daqmanager/private/createChannels - Added new channel ''%s'' to empty board.\n%s', gdm(dm.ptr).channels(indices(i)).name, getStackTraceString);
% fprintf(1, 'Added output channel %s:%s:''%s''\n', num2str(gdm(dm.ptr).channels(indices(i)).boardId), ...
%     num2str(gdm(dm.ptr).channels(indices(i)).channelId), gdm(dm.ptr).channels(indices(i)).name);
        
         %%%%COMMENTED OUT (VI050508A)
        %set(chan, 'OutputRange', [-10 10], 'UnitsRange', [-10 10]);%TO070606B - Enforce a default -10 to 10 V range. -- Tim O'Connor 7/6/06
        %%%%%%%%%%%%%%%%%%%%
        
% fprintf(1, '@daqmanager/private/createChannels - Set OutputRange and UnitsRange to [-10, 10]\n\n');
        end
    else
% fprintf(1, 'Adding input channel %s:%s:''%s''\n', num2str(gdm(dm.ptr).channels(indices(i)).boardId), ...
%     num2str(gdm(dm.ptr).channels(indices(i)).channelId), gdm(dm.ptr).channels(indices(i)).name);
        ai = gdm(dm.ptr).ais{gdm(dm.ptr).channels(indices(i)).boardId};
        %TO012706F - Any existing channels need to get restarted for allowMultistart. -- Tim O'Connor 1/27/06
        if strcmpi(get(ai, 'Running'), 'On') & gdm(dm.ptr).allowMultistart
            stop(ai);
        end
        %TO013106B: Make sure the same channel isn't added multiple times. -- Tim O'Connor 1/31/06
        if ~isempty(ai.Channel)
            if ismember(gdm(dm.ptr).channels(indices(i)).name, ai.Channel(:).ChannelName)
                error('Channel ''%s'' is already added to the board.', gdm(dm.ptr).channels(indices(i)).name);
            end
        end
        chan = addchannel(ai, gdm(dm.ptr).channels(indices(i)).channelId, gdm(dm.ptr).channels(indices(i)).name);
        set(chan, 'InputRange', [-10 10], 'SensorRange', [-10 10], 'UnitsRange', [-10 10]);%TO032006B - Enforce a default -10 to 10 V range. -- Tim O'Connor 3/21/06
    end
end
% for i = 1 : length(indices)
%     if gdm(dm.ptr).channels(indices(order(i))).ioFlag == 0
%         addOutputChannel(dm, gdm(dm.ptr).channels(indices(order(i))).boardId, gdm(dm.ptr).channels(indices(order(i))).channelId, gdm(dm.ptr).channels(indices(order(i))).name);
%     else
%         addInputChannel(dm, gdm(dm.ptr).channels(indices(order(i))).boardId, gdm(dm.ptr).channels(indices(order(i))).channelId, gdm(dm.ptr).channels(indices(order(i))).name);
%     end
% end

return;