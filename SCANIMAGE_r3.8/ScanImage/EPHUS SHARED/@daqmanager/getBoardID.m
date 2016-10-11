% @daqmanager/getBoardID
%
% SYNTAX
%  boardID = getBoardID(obj, channelName)
%   obj - The handle to the @daqmanager instance.
%   channelName - The name of the channel for which to retrieve the boardID.
%   boardID - The boardID for the named channel.
%
% USAGE
%
% NOTES
%
% CHANGES
%
% Created 12/30/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function boardID = getBoardID(this, channelName)

channel = getChannel(this, channelName);
if isempty(channel)
    error('Channel ''%s'' does not exist.', channelName);
end

boardID = channel.boardId;

return;