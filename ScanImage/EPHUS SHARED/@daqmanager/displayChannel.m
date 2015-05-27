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
%%           1/27/04 Tim O'Connor TO12704d: Add support for input channels.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function property = displayChannel(dm, varargin)
global gdm;

%fprintf(1, 'Channel\tState\tBoardId\tChannelId\tType\tData\n');
for i = 1 : length(varargin)
    index = getChannelIndex(dm, varargin{i});
    if index < 1
        error(sprintf('No channel found with identifier: ''%s''', varargin{i}));
    end
    
    %Name
    fprintf(1, 'Name: %s\n', gdm(dm.ptr).channels(index).name);
    
    %State
    if gdm(dm.ptr).channels(index).state == 0
        state = 'disabled';
    elseif gdm(dm.ptr).channels(index).state == 1
        state = 'enabled';
    elseif gdm(dm.ptr).channels(index).state == 2
        state = 'started';
    else
        state = '???';
    end
    fprintf(1, 'State: %s\n', state);
    
    %BoardId
    fprintf(1, 'BoardId: %s\n', num2str(gdm(dm.ptr).channels(index).boardId));
    
    %ChannelId
    fprintf(1, 'ChannelId: %s\n', num2str(gdm(dm.ptr).channels(index).channelId));
    
    %IO Type
    if gdm(dm.ptr).channels(index).ioFlag == 0
        type = 'analog-output';
    elseif gdm(dm.ptr).channels(index).ioFlag == 1
        type = 'analog-input';
    elseif gdm(dm.ptr).channels(index).ioFlag == 2
        type = 'digitalio';
    else
        type = '???';
    end
    fprintf(1, 'Type: %s\n', type);
    
    %Data
    if gdm(dm.ptr).channels(index).ioFlag == 0 %TO12704d - This only makes sense in the context of output channels.
        fprintf(1, 'Data: %s samples (%s seconds)\n', num2str(length(gdm(dm.ptr).channels(index).data)), ...
            num2str(length(gdm(dm.ptr).channels(index).data) / getAOProperty(dm, gdm(dm.ptr).channels(index).name, 'SampleRate')));
    end
    
    %Channel Properties
    fprintf(1, 'ChannelProperties:\n ');
%     displayCellArrayTableRowNames(gdm(dm.ptr).channels(index).chanProps);
    displayCellArrayTable(gdm(dm.ptr).channels(index).chanProps);
    
    %AO Properties
    fprintf(1, 'BoardProperties:\n ');
%     displayCellArrayTableRowNames(gdm(dm.ptr).channels(index).aoProps);
    displayCellArrayTable(gdm(dm.ptr).channels(index).aoProps);

    fprintf(1, '\n');
end

return;