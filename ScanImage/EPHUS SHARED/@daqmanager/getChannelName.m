% DAQMANAGER/getChannelName - Get the string associated with a hardware channel.
%
%  SYNTAX
%   channelName = getChannelName(this, boardId, channelId, ioFlag)
%   channelName = getChannelName(this, boardId, channelId)
%    this - DAQMANAGER
%    boardId - The board # of the channel being looked-up.
%    channelId - The channel # of the channel being looked-up.
%    ioFlag - 1 if the channel is an input channel, 0 if it's an output channel.
%    channelName - The name of the channel being looked-up.
%
% NOTES
%  If no ioFlag is specified, a comma-separated list of names may be returned, in the event
%  that there exists an input and an output channel with the same name.
%
% CHANGES
%  TO033005A: Allow I/O agnosticism. -- Tim O'Connor 3/30/05
%
% Created 2/10/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function channelName = getChannelName(this, boardId, channelId, varargin)
global gdm;

index = -1;
if ~isempty(varargin)
    index = getChannelIndex(this, boardId, channelId, varargin{1});
else
    index = getChannelIndex(this, boardId, channelId, 0);
    inIndex = getChannelIndex(this, boardId, channelId, 1);
    
    if index > -1
        index(2) = inIndex;
    else
        index = inIndex;
    end
end
if index < 1
    if ioFlag
        error('No input channel found with boardId %s and channelId %s.', num2str(boardId), num2str(channelId));
    else
        error('No output channel found with boardId %s and channelId %s.', num2str(boardId), num2str(channelId));
    end
end

channelName = gdm(this.ptr).channels(index(1)).name;
if length(index) > 1
    channelName = [channelName ', ' gdm(this.ptr).channels(index(2)).name];
end

return;
