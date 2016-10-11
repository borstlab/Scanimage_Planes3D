% DAQMANAGER/setChannelStartListener - Register a start listener for a specific channel.
%
% SYNTAX
%   setChannelStartListener(DAQMANAGER, channelName, listenerFunction, listenerID)
%     DAQMANAGER - object
%     channelName - The name of the channel to be bound.
%     listenerFunction - Function to get executed on channel start.
%     listenerID - A unique identifier (a number, a string, or an object implementing the `eq` method).
%
% NOTES:
%   If listenerFunction is a cell array or function handle, it will be passed the
%   channel name, daq object, and eventdata as the final 3 arguments.
%
%   If an empty listener function is specified, the listener is removed.
%
% CHANGES
%  TO080905G - Fixed missing brackets around cell array expansion. -- Tim O'Connor 8/9/05
%  TO112205C - Switch implementation to use @callbackManager. -- Tim O'Connor 11/22/05
%  TO062306B: @daqmanager/isChannel has been deprecated in favor of @daqmanager/hasChannel. -- Tim O'Connor 6/23/06
%
% Created 1/20/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function setChannelStartListener(this, channelName, listenerFunction, listenerID)
global gdm;

if ~hasChannel(this, channelName)
    error('Can not bind a channelStartListener for a channel that does not exist: %s', channelName);
end

if isempty(listenerFunction)
    if hasCallback(gdm(this.ptr).cbm, [channelName 'Stop'], listenerID)
        removeCallback(gdm(this.ptr).cbm, [channelName 'Stop'], listenerID);
    end
else
    addCallback(gdm(this.ptr).cbm, [channelName 'Start'], listenerFunction, listenerID);
end
%TO112205C
% validateCallbackSpec(listenerFunction);
% 
% index = [];
% if ~isempty(gdm(this.ptr).channelStartListener)
%     index = find(strcmpi({gdm(this.ptr).channelStartListener{:, 1}}, channelName));%TO080905G
% end
% 
% if isempty(index)
%     index = size(gdm(this.ptr).channelStartListener, 1) + 1;
%     gdm(this.ptr).channelStartListener{index, 1} = channelName;
% end
% 
% gdm(this.ptr).channelStartListener{index, 2} = listenerFunction;

return;