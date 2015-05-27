%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Print out information about the given channel(s).
%%
%%  displayChannel(dm, 'channelName', ...)
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function property = displayChannel(dm, varargin)
global gdm;

%fprintf(1, 'Channel\tState\tBoardId\tChannelId\tType\tData\n');
for i = 1 : length(varargin)
    channel = gdm(dm.ptr).channels(getChannelIndex(dm, varargin{i}));
    
    %Name
    fprintf(1, 'Name: %s\n', channel.name);
    
    %State
    if channel.state = 0
        state = 'disabled';
    elseif channel.state == 1
        state = 'enabled';
    elseif channel.state == 2
        state = 'started';
    else
        state = '???';
    end
    fprintf(1, 'State: %s\n', state);
    
    %BoardId
    fprintf(1, 'BoardId: %s\n', num2str(channel.boardId));
    
    %ChannelId
    fprintf(1, 'ChannelId: %s\n', num2str(channel.channelId));
    
    %IO Type
    if channel.ioFlag == 0
        type = 'analog-output';
    elseif channel.ioFlag == 1
        type = 'analog-input';
    elseif channel.ioFlag == 2
        type = 'digitalio';
    else
        type = '???';
    end
    fprintf(1, 'Type: %s\n', type);
    
    %Data
    fprintf(1, 'Data: %s samples\n', num2str(length(channel.data)));
    
    %Channel Properties
    fprintf(1, 'ChannelProperties: ');
    displayCellArrayTableRowNames(channel.chanProps);
    
    %AO Properties
    fprintf(1, 'BoardProperties: ');
    displayCellArrayTableRowNames(channel.aoProps);

    fprintf(1, '\n');
end

return;