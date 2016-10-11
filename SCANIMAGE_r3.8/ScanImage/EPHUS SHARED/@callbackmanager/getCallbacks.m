% @callbackManager/getCallbacks - Retrieve callbacks bound to an event.
%
%  SYNTAX
%   callbacks = getCallbacks(callbackmanager, event)
%    callbackmanager - @callbackmanager object instance.
%    event - A string (case insensitive), naming the event from which to retrieve callbacks.
%    callbacks - A cell array of the callbacks bound to the named event.
%
%  CHANGES
%   TO123005P - Moved the access of the callbackStruct, because the if statement was working on a non-existent variable. -- Tim O'Connor 12/30/05
%   TO123005Q - Changed it from looking at the last element to looking at the indexed element. -- Tim O'Connor 12/30/05
%
% Created 12/6/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function callbacks = getCallbacks(this, event)
global callbackmanagers;

index = [];
if ~isempty(callbackmanagers(this.ptr).callbacks)
    index = find(strcmpi({callbackmanagers(this.ptr).callbacks{:, 1}}, event));
end
if isempty(index)
    callbacks = {};
else
    callbackStruct = callbackmanagers(this.ptr).callbacks{index, 2};%TO123005P
    if isempty(callbackStruct)
        callbacks = {};
    else        
        for i = 1 : length(callbackStruct)
            callbacks{i} = callbackStruct(i).callbackSpec;
        end
    end
end

return;