% DAQMANAGER/removeChannelStopListener - Remove a stop listener for a specific channel.
%
% SYNTAX
%   removeChannelStopListener(DAQMANAGER, channelName, listenerID)
%     DAQMANAGER - object
%     channelName - The name of the channel to be bound.
%     listenerID - A unique identifier (a number, a string, or an object implementing the `eq` method).
%
% NOTES
%  See TO031606A.
%
% CHANGES
%  TO112205C - Switch implementation to use @callbackManager. -- Tim O'Connor 3/13/06
%  TO062306B: @daqmanager/isChannel has been deprecated in favor of @daqmanager/hasChannel. -- Tim O'Connor 6/23/06
%
% Created 1/20/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function setChannelStopListener(this, channelName, listenerID)
global gdm;

%TO062306B
if ~hasChannel(this, channelName)
    error('Can not bind a channelStopListener for a channel that does not exist: %s', channelName);
end

removeCallback(gdm(this.ptr).cbm, [channelName 'Stop'], listenerID);

return;