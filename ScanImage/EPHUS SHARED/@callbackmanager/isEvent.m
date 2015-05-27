% @callbackManager/isEvent - Determine if a specified event exists.
%
%  SYNTAX
%   hasEvent = isEvent(callbackmanager, eventName)
%    callbackmanager - @callbackmanager object instance.
%    eventName - An event name or a cell array listing events.
%    hasEvent - A boolean result, 1 if the event exists, 0 otherwise.
%
%  CHANGES
%
% Created 8/14/07 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2007
function events = isEvent(this, eventName)
global callbackmanagers;

events = any(ismember(eventName, {callbackmanagers(this.ptr).callbacks{:, 1}}));

return;