% DAQMANAGER/getChannelStartListener - Retrieve a start listener for a specific channel.
%
% SYNTAX
%   getChannelStartListener(DAQMANAGER, channelName)
%     DAQMANAGER - object
%     channelName - The name of the channel for which to retrieve a listener.
%
% RETURNS
%   listenerFunction - A function specification (string, function_handle, or cell array) or empty.
%
% CHANGES
%  TO112205C - Switch implementation to use @callbackManager. -- Tim O'Connor 11/22/05
%
% Created 1/20/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function listenerFunction = getChannelStartListener(this, channelName)
global gdm;
error('@daqmanager/getChannelStartListener - Obsoleted by TO112205C. Implementation to be rewritten.');
if ~isChannel(this, channelName)
    error('Can not find a channelStartListener for a channel that does not exist: %s', channelName);
end

index = [];
if ~isempty(gdm(this.ptr).channelStartListener)
    index = find(strcmpi(gdm(this.ptr).channelStartListener{:, 1}, channelName));
end

if isempty(index)
    listenerFunction = '';
else
    listenerFunction = gdm(this.ptr).channelStartListener{index, 2};
end

return;