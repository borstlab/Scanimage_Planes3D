% removeEvent - Remove an event that may have callbacks bound.
%
%  SYNTAX
%   removeEvent(callbackmanager, event)
%    callbackmanager - @callbackmanager object instance.
%    event - A string (case insensitive), naming the event to which to tie callbacks.
%
%  CHANGES
%   TO012006A - Fixed the cell array indexing. -- Tim O'Connor 1/20/06
%
% Created 11/22/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function removeEvent(this, event)
global callbackmanagers;

if isempty(callbackmanagers(this.ptr).callbacks)
    warning('Event does not exist: %s', event);
    return;
end

index = find(strcmpi({callbackmanagers(this.ptr).callbacks{:, 1}}, event));
if isempty(index)
    warning('Event does not exist: %s', event);
    return;
end

callbackmanagers(this.ptr).callbacks(index:end-1, 1) = callbackmanagers(this.ptr).callbacks(index+1:end, 1);%TO012006A
callbackmanagers(this.ptr).callbacks(index:end-1, 2) = callbackmanagers(this.ptr).callbacks(index+1:end, 2);%TO012006A
callbackmanagers(this.ptr).callbacks = callbackmanagers(this.ptr).callbacks(1:end-1, :);%TO012006A

return;