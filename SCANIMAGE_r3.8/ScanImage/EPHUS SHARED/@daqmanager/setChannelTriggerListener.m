% DAQMANAGER/setChannelTriggerListener - Register a trigger listener for a specific channel.
%
% SYNTAX
%   setChannelTriggerListener(DAQMANAGER, channelName, listenerFunction, listenerID)
%     DAQMANAGER - object
%     channelName - The name of the channel to be bound.
%     listenerFunction - Function to get executed on channel start.
%     listenerID - A unique identifier (a number, a string, or an object implementing the `eq` method).
%
% NOTES
%   If listenerFunction is a cell array or function handle, it will be passed the
%   channel name, daq object, and eventdata as the final 3 arguments.
%
%   See TO112205C.
%
%   If an empty listener function is specified, the listener is removed.
%
% CHANGES
%  TO123005O: Fixed typo, it was overwriting the 'Stop' events, because it said that instead of 'Trigger'. -- Tim O'Connor 12/30/05
%  TO062306B: @daqmanager/isChannel has been deprecated in favor of @daqmanager/hasChannel. -- Tim O'Connor 6/23/06
%
% Created 11/22/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function setChannelTriggerListener(this, channelName, listenerFunction, listenerID)
global gdm;

%TO062306B
if ~hasChannel(this, channelName)
    error('Can not bind a channelStartListener for a channel that does not exist: %s', channelName);
end

if isempty(listenerFunction)
    if hasCallback(gdm(this.ptr).cbm, [channelName 'Trigger'], listenerID)
        removeCallback(gdm(this.ptr).cbm, [channelName 'Trigger'], listenerID);
    end
else
    addCallback(gdm(this.ptr).cbm, [channelName 'Trigger'], listenerFunction, listenerID);
end

return;