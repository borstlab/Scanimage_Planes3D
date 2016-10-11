% fireEvent - Fire a given event.
%
%  SYNTAX
%   fireEvent(callbackmanager, event)
%   fireEvent(callbackmanager, event, argument, ...)
%    callbackmanager - @callbackmanager object instance.
%    event - A string (case insensitive) or cell array, naming the event(s) whose callback(s) to execute.
%    argument - Any variable, to be passed on to all callbacks that are implmented as function_handles or cell arrays.
%               Multiple arguments may be specified.
%
%  NOTES
%
%  CHANGES
%   TO121505A: Warn if arguments are passed to a 'char' based callback which can not process them. -- Tim O'Connor 12/15/05
%   TO010506D: Implemented the enable field. -- Tim O'Connor 2/24/06 (The implementation was left unfinished until 2/24/06 for some unkown reason.
%   TO081506B: Make sure the 'Invalid event' error message always prints a stack trace. -- Tim O'Connor 8/15/06
%   TO101707G: Updated to use ischar, instead of strcmpi(class(...), 'char').-- Tim O'Connor 10/17/07
%   TO113007C: Created getLastErrorStack. -- Tim O'Connor 11/30/07
%   TO073008F: Allow multiple events to be fired at once (vectorize this function). -- Tim O'Connor 7/30/08
%
% Created 5/12/05 - Tim O'Connor
%
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function fireEvent(this, event, varargin)
global callbackmanagers;

% fprintf(1, '%s - @callbackManager/fireEvent: ''%s''\n', datestr(now), event);
if iscell(event)
    for i = 1 : length(event)
        fireEvent(this, event{i}, varargin{:});
    end
end

index = find(strcmpi({callbackmanagers(this.ptr).callbacks{:, 1}}, event));
if isempty(index)
    error('Invalid event: %s\n%s', event, getStackTraceString);%TO081506B
end

if ~callbackmanagers(this.ptr).enable
    return;
end

callbacks = callbackmanagers(this.ptr).callbacks{index, 2};
if isempty(callbacks)
    return;
end

[priorities order] = sort([callbacks(:).priority]);
for i = 1 : length(callbacks)
    try
        switch lower(class(callbacks(order(i)).callbackSpec))
            case 'char'
                %TO121505A
                if ~isempty(varargin)
                    id = callbacks(order(i)).id;
                    if isnumeric(id)
                        id = num2str(id);
                    elseif ~ischar(id)
                        id = ['Instance of ' class(id)];
                    end
                    warning('Arguments passed to a callback which does not support them: %s\n Event: %s\n ID: %s\n%s', callbacks(order(i)).callbackSpec, event, id, getStackTraceString);
                end
                eval(callbacks(order(i)).callbackSpec);
            case 'function_handle'
                feval(callbacks(order(i)).callbackSpec, varargin{:});
            case 'cell'
                feval(callbacks(order(i)).callbackSpec{:}, varargin{:});
            otherwise
                error('Invalid callbackSpec: s', class(callbacks(order(i)).callbackSpec));
        end
    catch
        id = callbacks(order(i)).id;
        if isnumeric(id)
            id = num2str(id);
        elseif ~ischar(id)
            id = ['Instance of ' class(id)];
        end
        fprintf(2, '%s @callbackManager/fireEvent - Failed to execute callback: %s\n Event: %s\n ID: %s\n%s\n', datestr(now), lasterr, event, id, getLastErrorStack);
    end
end

return;
