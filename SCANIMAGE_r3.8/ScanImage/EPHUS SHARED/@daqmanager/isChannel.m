%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  See if a channel exists.
%%
%%  Returns 0 if the channel does not exist, a number
%%  greater than 0 otherwise.
%%
%%  logical = hasChannel(OBJ, name)
%%  logical = hasChannel(OBJ, boardId, channelId)
%%
%%  Created - Tim O'Connor 2/26/04
%%
%%  Changed:
%%   TO062306B: Deprecated due to possible namespace conflicts, as recommended by Vijay Iyer. -- Tim O'Connor 6/23/06
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function yOrN = isChannel(dm, varargin)

warning('DEPRECATED: @daqmanager/isChannel may cause namespace conflicts, use @daqmanager/hasChannel instead.\n%s', getStackTraceString);

yOrN = hasChannel(dm, varargin);