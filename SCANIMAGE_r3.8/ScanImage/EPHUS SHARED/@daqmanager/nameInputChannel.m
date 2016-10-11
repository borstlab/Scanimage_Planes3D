%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Bind a name to a channelId-boardId pair.
%%
%%  Channel names CAN NOT be overwritten. The channel must first be denamed,
%%  before it can be bound to another channelId-boardId pair.
%%
%%  [OBJ channel] = nameInputChannel(OBJ, boardId, channelId, 'name')
%%
%%  Created - Tim O'Connor 11/27/04
%%
%%  Changed:
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function varargout = nameInputChannel(dm, boardId, channelId, name)
global gdm;

nameChannel(dm, boardId, channelId, 1, name);

return;