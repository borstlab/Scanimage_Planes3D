% addEvent - Add an event that may have callbacks bound.
%
%  SYNTAX
%   addCallback(callbackmanager, event)
%   addCallback(callbackmanager, event, documentation)
%    callbackmanager - @callbackmanager object instance.
%    event - A string (case insensitive), naming the event to which to tie callbacks.
%    documentation - A string to be used to document this event.
%                    Specifically, this is meant to be viewable with the userFcns gui.
%
%  CHANGES
%   TO053008E - Add optional integrated event documentation. -- Tim O'Connor 5/30/08
%
% Created 5/18/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function addEvent(this, event, varargin)
global callbackmanagers;

if ~isempty(callbackmanagers(this.ptr).callbacks)
    index = find(strcmpi({callbackmanagers(this.ptr).callbacks{:, 1}}, event));
    if ~isempty(index)
        warning('Event already exists: %s', event);
        return;
    end
end
callbackmanagers(this.ptr).callbacks{size(callbackmanagers(this.ptr).callbacks, 1) + 1, 1} = event;
callbackmanagers(this.ptr).callbacks{size(callbackmanagers(this.ptr).callbacks, 1), 2} = [];
if isempty(varargin)
    callbackmanagers(this.ptr).callbacks{size(callbackmanagers(this.ptr).callbacks, 1), 3} = '';
else
    callbackmanagers(this.ptr).callbacks{size(callbackmanagers(this.ptr).callbacks, 1), 3} = varargin{1};
end

return;