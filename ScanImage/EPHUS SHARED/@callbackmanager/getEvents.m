% @callbackManager/getEvents - Retrieve a list of events.
%
%  SYNTAX
%   events = getEvents(callbackmanager)
%    callbackmanager - @callbackmanager object instance.
%    events - A cell array listing all events handled by this object.
%
%  CHANGES
%
% Created 12/6/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function events = getEvents(this)
global callbackmanagers;

events = {callbackmanagers(this.ptr).callbacks{:, 1}};

return;