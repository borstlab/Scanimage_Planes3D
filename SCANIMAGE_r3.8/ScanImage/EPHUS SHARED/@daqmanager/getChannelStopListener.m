% DAQMANAGER/getChannelStopListener - Retrieve a start listener for a specific channel.
%
% SYNTAX
%   getChannelStopListener(DAQMANAGER, channelName)
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
function listenerFunction = getChannelStopListener(this, channelName)
global gdm;
error('@daqmanager/getChannelStopListener - Obsoleted by TO112205C. Implementation to be rewritten.');
if ~isChannel(this, channelName)
    error('Can not find a channelStopListener for a channel that does not exist: %s', channelName);
end

index = [];
if ~isempty(gdm(this.ptr).channelStopListener)
    index = find(strcmpi(gdm(this.ptr).channelStopListener{:, 1}, channelName));
end

if isempty(index)
    listenerFunction = '';
else
    listenerFunction = gdm(this.ptr).channelStopListener{index, 2};
end

return;