% @callbackManager/getCallbacksAsStrings - Retrieve strings representing the callbacks.
%
%  SYNTAX
%   callbacks = getCallbacksAsStrings(callbackmanager, event)
%    callbackmanager - @callbackmanager object instance.
%    event - A string (case insensitive), naming the event from which to retrieve callbacks.
%    callbacks - A cell array of strings representing the callbacks bound to the named event.
%
%  NOTES
%   The strings will be either the evalable string or the function name (if it is a function handle or cell array).
%
%  CHANGES
%   TO123005Q - Changed it from looking at the last element to looking at the indexed element. -- Tim O'Connor 12/30/05
%   TO083107C - Add an '@' to the string for function_handle callbacks and a '{@, ...}' to cell array callbacks, to make them easy to distinguish. -- Tim O'Connor 8/31/07
%   TO090307A - Back out TO083107C for now, because it screws up saving of the userFcn gui's configuration. -- Tim O'Connor 9/3/07
%
%  CREDITS
% Created 12/6/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function callbackStrings = getCallbacksAsStrings(this, event)
global callbackmanagers;

callbackStrings = {};
index = [];
if ~isempty(callbackmanagers(this.ptr).callbacks)
    index = find(strcmpi({callbackmanagers(this.ptr).callbacks{:, 1}}, event));
end
if isempty(index)
    callbacks = {};
else
    callbackStruct = callbackmanagers(this.ptr).callbacks{index, 2};
    if isempty(callbackStruct)
        callbacks = {};
    else
        for i = 1 : length(callbackStruct)
            switch lower(class(callbackStruct(i).callbackSpec))
                case 'char'
                    callbackStrings{i} = callbackStruct(i).callbackSpec;
                case 'function_handle'
                    %TO083107C %TO090307A
                    % callbackStrings{i} = ['@' func2str(callbackStruct(i).callbackSpec)];
                    callbackStrings{i} = func2str(callbackStruct(i).callbackSpec);
                case 'cell'
                    %TO083107C %TO090307A
                    % callbackStrings{i} = ['{@' func2str(callbackStruct(i).callbackSpec{1}) ', ...}'];
                    callbackStrings{i} = func2str(callbackStruct(i).callbackSpec{1});
                otherwise
                    warning('Unrecognized callback type: ''%s''', class(callbackStruct(i).callbackSpec));
            end
        end
    end
end

return;