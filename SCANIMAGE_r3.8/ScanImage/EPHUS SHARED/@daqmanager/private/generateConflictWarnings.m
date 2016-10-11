%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Find conflicts in board properties for the specified channels.
%%  Returns a cell array of messages, one for each conflict. If none
%%  are found, it returns an empty cell array.
%%
%%  {warnings} = generateWarnings(OBJ, channelNames)
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%           1/27/05 Tim O'Connor TO012705b: Fixed bug that looked at aoProps and ignored aiProps.
%%           1/27/05 Tim O'Connor TO012705c: Optimized conflict search, only compare similar io types.
%%           6/27/05 Tim O'Connor TO062705H: Fixed to properly account for AI objects, not just AO objects.
%%           6/27/05 Tim O'Connor TO062705G: This needed to be more robust across data types.
%%           1/6/06  Tim O'Connor TO010606E: Optimization(s).
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function messages = generateWarnings(dm, channelNames)
global gdm;

messages = {};
checked = [];
channels = {};

if strcmpi(class(channelNames), 'char')
    channelNames = {channelNames};
end

for i = 1 : length(channelNames)
    index = getChannelIndex(dm, channelNames{i});
    
    if index < 1
        errMsg = sprintf('No channel found with name ''%s''.', channelNames(i));
        error(errMsg);
    end
    
    if ~ismember(gdm(dm.ptr).channels(index).boardId, checked)
        channels{length(channels) + 1} = gdm(dm.ptr).channels(index);
    end
end

messages = iterate(channels);

return;

%-------------------------------------------------------
function messages = iterate(channels)

messages = {};
checked = {};

%TO062705H: Fixed to properly account for AI objects, not just AO objects.
for i = 1 : length(channels) - 1
    if channels{i}.state > 0%It only matters if the channel is enabled/started.
        %Check each property of each channel.
        if channels{i}.ioFlag == 0
            x = size(channels{i}.aoProps, 1);
        else
            x = size(channels{i}.aiProps, 1);
        end
        for j = 1 : x
            if channels{i}.ioFlag == 0
                key = channels{i}.aoProps{j, 1};
            else
                key = channels{i}.aiProps{j, 1};
            end

            %Don't check the same value multiple times.
            if ~ismember(key, checked)
                msgs = findConflicts(channels{i}, channels, key);
                if ~isempty(msgs)
                    messages = cat(2, messages, msgs);
                end
                checked{length(checked) + 1} = key;
            end
        end
    end
end

return;

%-------------------------------------------------------
function messages = findConflicts(channel, channels, key)

%Compare the value associated with 'key' on 'channel' against
%any values with mathing keys in 'channels'. Count the matches 
%and generate a warning for each one.
messages = {};
caught = {};

for i = 1 : length(channels)

    if channels{i}.state > 0 & ... %Only check enabled/started channels.
            (channels{i}.boardId == channel.boardId) & ... %Only compare channels on the same board.
            ~strcmpi(channel.name, channels{i}.name)  & ... %Don't check the channel against itself.
            channel.ioFlag == channels{i}.ioFlag %Only compare when they are the same io type. TO012705c

        %TO012705b - Moved the 'index = ...' expression into the if/else block, it had only been considering aoProps before, not aiProps.
        if channel.ioFlag == 0
            index = getRowIndex(channels{i}.aoProps, key);
            val1 = channels{i}.aoProps{index, 2};
            val2 = channel.aoProps{getRowIndex(channel.aoProps, key), 2};
        elseif channel.ioFlag == 1
            index = getRowIndex(channels{i}.aiProps, key);
            val1 = channels{i}.aiProps{index, 2};
            val2 = channel.aiProps{getRowIndex(channel.aiProps, key), 2};            
        end

        %This gets rather ugly, but it's good to be thorough about checking this stuff.
        if index > -1 & strcmp(class(val1), class(val2))
            %We found two keys with values of the same class.
            try
                %TO062705G - This needed to be more robust across data types. -- Tim O'Connor 6/27/05
                conflictFound = 0;
                if isnumeric(val1)
                    conflictFound = (length(val1) ~= length(val2));
                    if ~conflictFound
                        conflictFound = (val1 ~= val2);
                    end
                else
                    switch lower(class(val1))
                        case 'char'
                            conflictFound = ~strcmp(val1, val2);
                        case 'function_handle'
                            conflictFound = ~strcmp(func2str(val1), func2str(val2));
                        case 'cell'
                            conflictFound = (length(val1) ~= length(val2));
                            if ~conflictFound
                                for j = 1 : length(val1)
                                    conflictFound = (length(val1{j}) ~= length(val2{j})) & ~strcmp(class(val1{j}), class(val2{j}));
                                end
                            end
                        otherwise
                            conflictFound = (val1 == val2);
                    end
                end

                if conflictFound
                    %The values of the same class turned out to be non-equal to each other.
                    %This is the source of a conflict.
                    messages{length(messages) + 1} = sprintf('Conflict found in acquisition board properties between channels ''%s'' and ''%s'' for property ''%s''.', channel.name, channels{i}.name, key);
                    if channel.ioFlag == 0
                        messages{length(messages) + 1} = sprintf('\n %s\n %s', channel.name, channel.aoProps{getRowIndex(channel.aoProps, key), 3});
                        messages{length(messages) + 1} = sprintf('\n %s\n %s', channels{i}.name, channels{i}.aoProps{index, 3});
%                         messages{length(messages) + 1} = sprintf('\n ''%s'' last set event - %s', channel.name, channel.aoProps{getRowIndex(channel.aoProps, key), 3});
%                         messages{length(messages) + 1} = sprintf('\n ''%s'' last set event - %s', channels{i}.name, channels{i}.aoProps{index, 3});
                    elseif channel.ioFlag == 1
                        messages{length(messages) + 1} = sprintf('\n %s\n', channel.aiProps{getRowIndex(channel.aiProps, key), 3});        
                        messages{length(messages) + 1} = sprintf('\n %s\n', channels{i}.aiProps{index, 3});
%                         messages{length(messages) + 1} = sprintf('\n ''%s'' last set event - %s', channel.name, channel.aiProps{getRowIndex(channel.aiProps, key), 3});        
%                         messages{length(messages) + 1} = sprintf('\n ''%s'' last set event - %s', channels{i}.name, channels{i}.aiProps{index, 3});
                    end
                end
            catch
                caught{length(caught) + 1} = sprintf('Failed to compare values for property ''%s''.', key);
            end
        end
    end
end

if length(messages) > 1 & ~isempty(caught)
    %Only display these errors if other problems exist, since these will almost always come up for incomparable types.
    messages = cat(2, messages, caught);
end

return;