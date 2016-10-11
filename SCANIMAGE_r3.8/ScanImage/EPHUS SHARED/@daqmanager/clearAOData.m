%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Remove any queued data from the analog output object.
%%
%%  [data_length ...] = clearAOData(OBJ, channelName)
%%  [data_length ...] = clearAOData(OBJ, boardId)
%%
%%  Clear the object containing the named channel.
%%
%%  Returns the length of all the channels that got cleared, in order.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%           1/27/04 Tim O'Connor TO12704c: Wasn't clearing the data for all channels on the AO object.
%%           9/7/06 Tim O'Connor TO090706C: Some serious problems were fixed. See details in code.
%%           9/8/06 Tim O'Connor TO090806B: Do not clear the @daqmanager buffer.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cleared = clearAOData(dm, identifier)

if isnumeric(identifier)
    cleared = clearAOByBoardId(dm, identifier);
elseif strcmpi(class(identifier), 'char')
    cleared = clearAOByChannelName(dm, identifier);
else
    error(sprintf('Invalide identifier type for clearAOData: ''%s''', class(identifier)));
end

return;

%-------------------------------------------------------
function cleared = clearAOByChannelName(dm, channelName)
global gdm;

%TO090706C - This had been passing in the channel structure, adding `.boardId` to the end is the correct thing to do.
cleared = clearAOByBoardId(dm, gdm(dm.ptr).channels(getChannelIndex(dm, channelName)).boardId);

return;

%-------------------------------------------------------
function cleared = clearAOByBoardId(dm, boardId)
global gdm;

%TO090706C - Since this is now a growing array, it needs to be pre-declared.
channels = [];
%Iterate over all channels.
for i = 1 : length(gdm(dm.ptr).channels)
    %Clear channels on the specified board.
    %TO090706C - This had been testing a taughtology. It should compare against the boardId argument.
    %if gdm(dm.ptr).channels(i).boardId == gdm(dm.ptr).channels(i).boardId
    if gdm(dm.ptr).channels(i).boardId == boardId
        %TO090706C - Need to make sure only outputs are considered.
        if gdm(dm.ptr).channels(i).ioFlag == 0
            %Track how much data was deleted and on which channel.
            %TO090706C - This can not use `i` to index the channels array, on the left hand side.
            channels(1, size(channels, 2) + 1) = gdm(dm.ptr).channels(i).channelId;
            channels(2, size(channels, 2)) = length(gdm(dm.ptr).channels(i).data);
% fprintf(1, '@daqmanager/clearAOData: Clearing buffers for channel ''%s''...\n', gdm(dm.ptr).channels(i).name);
            %gdm(dm.ptr).channels(i).data = [];%TO090806B
        end
    end
end

%Sort the data lengths by channelId.
sort(channels);
cleared = channels(2, :);

return;