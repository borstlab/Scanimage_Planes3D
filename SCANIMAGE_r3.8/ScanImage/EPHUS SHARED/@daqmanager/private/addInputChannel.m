%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Add a named channel.
%%
%%  [OBJ CHANNEL] = addInputChannel(OBJ, BOARDID, CHANNELID, 'NAME')
%%
%%  BOARDID - Which output board to add the channel to.
%%
%%  CHANNELID - Which output channel to add.
%%
%%  NAME - A unique string identifying this channel.
%%
%%  Created - Tim O'Connor 11/27/04
%%
%%  Changed:
%%   TO012005b - Fix direct indexing into AO/AI array. -- Tim O'Connor 1/20/05
%%   TO010606E - Optimization(s). -- Tim O'Connor 1/6/06
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function varargout = addInputChannel(dm, boardId, channelId, name)
global gdm;
% fprintf(1, '@daqmanager/addInputChannel - %s:%s - ''%s''\n', num2str(boardId), num2str(channelId), name);
%Make sure the boardId is okay.
if boardId > length(gdm(dm.ptr).ais)
    errmsg = sprintf('boardId %s out of range 1-%s.', num2str(boardId), num2str(length(gdm(dm.ptr).ais)));
    error(errmsg);
end

ai = getAI(dm, boardId);%TO012005b

%TO010606E - While this is a nice check, it's not entirely necessary and is costing 30ms.
%Make sure the channelId is okay.
% info = daqhwinfo(ai);%TO012005b
% if channelId > info.TotalChannels
%     errmsg = sprintf('channelId %s is not in the allowed set of [%s].', num2str(channelId), num2str(info.ChannelIDs));
%     error(errmsg);
% end

%Make sure the name exists.
if isempty(name)
    error('Channel name must not be empty.');
end

%Check for uniqueness.
%TO010606E - Try to do a single subsref into the Channel field array.
if ~isempty(ai.Channel)
    channelIds = [ai.Channel(:).HwChannel];
    if length(channelIds) > 1
        channelIds = [channelIds{:}];
    end
    conflicts = find(channelIds == channelId);
    if ~isempty(conflicts)
        errMsg = sprintf('Channel %s already in use on board %s.', mat2str(channelIds(conflicts)), num2str(boardId));
        error(errMsg);
    end
end

%Create the channel.
if strcmpi(get(ai, 'Running'), 'On')%TO012005b
    stop(ai);%TO012005b
end
ch = addchannel(ai, channelId, name);%TO012005b

%Marshal the return arguments.
if nargout >= 1
    varargout{1} = dm;
    
    if nargout == 2
        varargout{2} = ch;
    end
end
        

return;