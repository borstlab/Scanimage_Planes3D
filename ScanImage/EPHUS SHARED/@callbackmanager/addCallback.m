% @callbackManager/addCallback - Add a callback to be bound to an event.
%
%  SYNTAX
%   addCallback(callbackmanager, event, callbackSpec, callbackID)
%   addCallback(callbackmanager, event, callbackSpec, callbackID, priority)
%    callbackmanager - @callbackmanager object instance.
%    event - A string (case insensitive), naming the event to which to tie this callback.
%    callbackSpec - A valid callback (function_handle, cell array with a function_handle as the first argument, evalable string).
%    callbackID - A unique identifier (a number, a string, or an object implementing the `eq` method).
%    priority - Any number between 1 and 10, such that the lower the number, the more likely this callbackSpec is to be executed first (no gaurantees). (Default: 5)
%
% CHANGES
%   VI060308A - Handle option to display callback binding to command-line 
%
% CREDITS
% Created 5/12/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function addCallback(this, event, callbackSpec, callbackID, varargin)
global callbackmanagers;

if ~isnumeric(callbackID) && ~ischar(callbackID)
    if ~ismethod(callbackID, 'eq')
        error('Invalid callbackID. Must be a string, a number, or an object implementing the `eq` method. callbackID: %s', class(callbackID));
    end
end

priority = 5;
if ~isempty(varargin)
    priority = varargin{1};
end

%Argument error checking
if priority < 1 || priority > 10
    error('Priority out of range: %s', num2str(priority));
end

index = find(strcmpi({callbackmanagers(this.ptr).callbacks{:, 1}}, event));
if isempty(index)
    error('Invalid event: %s', event);
end

validateCallbackSpec(callbackSpec);
if callbackmanagers(this.ptr).handlesOnly
    if ischar(callbackSpec)
        error('This callback manager only allows function_handle based callbacks.');
    end
end

callbacks = callbackmanagers(this.ptr).callbacks{index, 2};
arrayIndex = length(callbacks) + 1;
for i = 1 : length(callbacks)
    if strcmpi(class(callbacks(i).id), class(callbackID))
        if ischar(callbackID)
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

%   callbacks - A Nx2 cell array. The first column is a string (representing the event), the second column contains 
%               a structure array.
%       nested structure:
%         id - A unique identifier for the callback, to facilitate deletion.
%         callbackSpec - The actual callback (a function_handle, a cell array whose first element is a function_handle, or a string).
%         priority - A priority specifier, to help in ordering callbacks during execution.
callbacks(arrayIndex).id = callbackID;
callbacks(arrayIndex).callbackSpec = callbackSpec;
callbacks(arrayIndex).priority = priority;

callbackmanagers(this.ptr).callbacks{index, 2} = callbacks;

return;
