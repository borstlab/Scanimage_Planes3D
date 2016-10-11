%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Unbind a name from a channelId-boardId pair.
%%
%%  Channel names CAN NOT be overwritten. The channel must first be denamed,
%%  before it can be bound to another channelId-boardId pair.
%%
%%  channelName = denameInputChannel(OBJ, boardId, channelId, ioFlag)
%%  channelName = denameInputChannel(OBJ, name)
%%
%%  Created - Tim O'Connor 11/27/04
%%
%%  Changed:
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function channelName = denameInputChannel(dm, varargin)
global gdm;

if length(varargin) == 2
    denameChannel(dm, varargin{1}, varargin{2}, 1);
else
    denameChannel(dm, varargin{1});
end

return;