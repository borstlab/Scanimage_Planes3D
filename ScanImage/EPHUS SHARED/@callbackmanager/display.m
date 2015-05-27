% callbackmanager/display - Print the object to the screen.
%
%  SYNTAX
%   display(cbm)
%    cbm - A @callbackmanager instance.
%
%  USAGE
%
%  CHANGES
%
% Created 10/16/07 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2007
function display(this)
global callbackmanagers;

fprintf(1, '@callbackmanager v0.2\nObjectPointer: %s\n', num2str(this.ptr));

fprintf(1, '\thandlesOnly: %s\n', num2str(callbackmanagers(this.ptr).handlesOnly));
fprintf(1, '\tenable: %s\n', num2str(callbackmanagers(this.ptr).enable));
if isempty(callbackmanagers(this.ptr).callbacks)
    fprintf(1, '\tNO_EVENTS\n');
else
    events = getEvents(this);
    for i = 1 : length(events)
        fprintf(1, '\tEvent ''%s'':\n', events{i});
        callbacks = getCallbacksAsStrings(this, events{i});
        if isempty(callbacks)
            fprintf(1, '\t\tNO_CALLBACKS\n');
        else
            for j = 1 : length(callbacks)
                fprintf(1, '\t\t%s\n', callbacks{j});
            end
        end
    end
end

return;