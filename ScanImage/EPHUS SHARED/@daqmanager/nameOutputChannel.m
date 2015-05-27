%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Bind a name to a channelId-boardId pair.
%%
%%  Channel names CAN NOT be overwritten. The channel must first be denamed,
%%  before it can be bound to another channelId-boardId pair.
%%
%%  [OBJ channel] = nameOutputChannel(OBJ, boardId, channelId, 'name')
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%           1/27/04 Tim O'Connor TO12704d: Add support for input channels.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function varargout = nameOutputChannel(dm, boardId, channelId, name)
global gdm;
% fprintf('@daqmanager/nameOutputChannel(this, %s, %s, %s)\n%s\n', num2str(boardId), num2str(channelId), name, getStackTraceString);
nameChannel(dm, boardId, channelId, 0, name);

return;