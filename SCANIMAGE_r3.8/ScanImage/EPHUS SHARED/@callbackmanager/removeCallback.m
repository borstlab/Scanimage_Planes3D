% removeCallback - Remove a callback that has been bound to an event.
%
% SYNTAX
%   removeCallback(callbackmanager, event, callbackID)
%    callbackmanager - @callbackmanager object instance.
%    event - A string (case insensitive), naming the event to which to this callback is tied.
%    callbackID - A unique identifier (a number, a string, or an object implementing the `eq` method).
%
% CHANGES
%
% CREDITS
% Created 5/12/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function removeCallback(this, event, callbackID)
global callbackmanagers;

index = find(strcmpi({callbackmanagers(this.ptr).callbacks{:, 1}}, event));
if isempty(index)
    error('Invalid event: %s', event);
end

callbacks = callbackmanagers(this.ptr).callbacks{index, 2};
arrayIndex = [];
for i = 1 : length(callbacks)
    if strcmpi(class(callbacks(i).id), class(callbackID))
        if strcmpi(class(callbackID), 'char')
            if strcmpi(callbacks(i).id, callbackID)
                arrayIndex = i;
                break;
            end
        else
            if callbacks(i).id == callbackID
                arrayIndex = i;
                break;
            end
        end
    end
end

if isempty(arrayIndex)
    msg = 'Invalid callbackID: ';
    if isnumeric(callbackID)
        msg = sprintf('%s: ''%s''', msg, num2str(callbackID));
    elseif strcmpi(class(callbackID), 'char')
        msg = sprintf('%s: ''%s''', msg, callbackID);
    else
        msg = sprintf('%s: object instance of type ''%s''', msg, class(callbackID));
    end
    
    error(msg);
end

callbacks(1 : arrayIndex - 1) = callbacks(1 : arrayIndex - 1);
callbacks(arrayIndex : end - 1) = callbacks(arrayIndex + 1 : end);
callbacks = callbacks(1 : end - 1);

callbackmanagers(this.ptr).callbacks{index, 2} = callbacks;

return;