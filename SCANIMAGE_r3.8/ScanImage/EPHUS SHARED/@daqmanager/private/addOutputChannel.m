%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Add a named channel.
%%
%%  [OBJ CHANNEL] = addOutputChannel(OBJ, BOARDID, CHANNELID, 'NAME')
%%
%%  BOARDID - Which output board to add the channel to.
%%
%%  CHANNELID - Which output channel to add.
%%
%%  NAME - A unique string identifying this channel.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%           4/21/04 Tim O'Connor TO042104a: Fix Matlab DAQ bug, see startChannel.m for details.
%%           TO012005b - Fix direct indexing into AO/AI array. -- Tim O'Connor 1/20/05
%%           11/22/05 Tim O'Connor TO112205A: Augmentation of the TO042104a fix.
%%           1/6/05 Tim O'Connor TO010406A: Take into account the change of board IDs from numbers to strings between Traditional NI-DAQ and DAQmx.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function varargout = addOutputChannel(dm, boardId, channelId, name)
global gdm;

% fprintf(1, '@daqmanager/addOutputChannel - %s:%s - ''%s''\n', num2str(boardId), num2str(channelId), name);
%Make sure the boardId is okay.
if boardId > length(gdm(dm.ptr).aos)
    errmsg = sprintf('boardId %s out of range 1-%s.', num2str(boardId), num2str(length(gdm(dm.ptr).aos)));
    error(errmsg);
end

ao = getAO(dm, boardId);%TO012005b

%Make sure the channelId is okay.
info = daqhwinfo(ao);%TO012005b
if isempty(find(info.ChannelIDs == channelId))
    errmsg = sprintf('channelId %s is not in the allowed set of [%s].', num2str(channelId), num2str(info.ChannelIDs));
    error(errmsg);
end

%Make sure the name exists.
if isempty(name)
    error('Channel name must not be empty.');
end

%Check for uniqueness.
for i = 1 : length(ao.Channel)%TO012005b
    if ao.Channel(i).HwChannel == channelId
        errMsg = sprintf('Channel %s already in use on board %s.', num2str(channelId), num2str(boardId));
        error(errMsg);
    end
end

%Create the channel.
%By using the form of addchannel that Matlab discourages, specifying an
%index (hardcoded as 1, in this case), it seems to have solved the problem
%with the TransferMode property. -- Tim O'Connor 4/21/04 TO042104a
%The problem popped up again, but is solved by changing the hardcoded 1 to be min(max([ao.Channel(:).ChannelIndex] + 1), channelId + 1). -- Tim O'Connor 11/22/05 TO112205A
%Sorting the calls, to increase with channelId is the ultimate solution (see @daqmanager/startChannel/createChannels).
% ch = addchannel(gdm(dm.ptr).aos(boardId), channelId, name);
if strcmpi(get(ao, 'Running'), 'On')%TO012005b
    stop(ao);%TO012005b
end
if ~isempty(ao.Channel)
    ch = addchannel(ao, channelId, min(max([ao.Channel(:).Index] + 1), channelId + 1), name);%TO012005b
else
    ch = addchannel(ao, channelId, 1, name);%TO012005b
end

%Marshal the return arguments.
if nargout >= 1
    varargout{1} = dm;
    
    if nargout == 2
        varargout{2} = ch;
    end
end
        

return;