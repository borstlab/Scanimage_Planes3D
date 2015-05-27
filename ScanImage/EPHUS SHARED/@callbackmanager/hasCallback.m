% hasCallback - Test for the existence of a specific callback on a specified event.
%
%  SYNTAX
%   callbackExists = hasCallback(callbackmanager, event, callbackID)
%    callbackmanager - @callbackmanager object instance.
%    event - A string (case insensitive), naming the event to which to this callback is tied.
%    callbackID - A unique identifier (a number, a string, or an object implementing the `eq` method).
%    callbackExists - 1 if the callbackID is present on the given event, 0 otherwise.
%
%  NOTES
%   This will also return false if the event does not exist.
%
%  CHANGES
%
% Created 5/12/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function callbackExists = hasCallback(this, event, callbackID)
global callbackmanagers;

callbackExists = 0;

index = find(strcmpi({callbackmanagers(this.ptr).callbacks{:, 1}}, event));
if isempty(index)
    return;
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
    return;
end

callbackExists = 1;

return;