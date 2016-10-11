%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Unbind a name from a channelId-boardId pair.
%%
%%  Channel names CAN NOT be overwritten. The channel must first be dename,
%%  before it can be bound to another channelId-boardId pair.
%%
%%  channelName = denameOutputChannel(OBJ, boardId, channelId, ioFlag)
%%  channelName = denameOutputChannel(OBJ, name)
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
function channelName = denameOutputChannel(dm, varargin)
global gdm;

if length(varargin) == 2
    denameChannel(dm, varargin{1}, varargin{2}, 0);
else
    denameChannel(dm, varargin{1});
end

return;