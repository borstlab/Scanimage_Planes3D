function wb = waitbarWithCancel(varargin)
% waitbarWithCancel - Returns a handle to a waitbar object.
% 
% This takes all the same arguments as waitbar, except that it sets the 
% 'CreateCancelBtn' property.
%
% Inside your loop, the one that updates the waitbar, you should call
% isWaitbarCancelled on this handle to see if it has been cancelled.
%
% NOTE
%    To get rid of this object, use `delete`, not `close`.
%
% See Also waitbar, isWaitbarCancelled
%
% CHANGES
%  TO090806D - Use a timer to make sure the waitbar disappears (eventually), in case there are errors. -- Tim O'Connor 9/8/06
%
% Created: Tim O'Connor 6/11/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004

%It must have a name, in order to specify a cancel button.
%Matlab defaults the name to 'Waitbar', so do that here too.
if length(varargin) == 1
    varargin{2} = 'Waitbar';
end

wb = waitbar(varargin{:}, 'CreateCancelBtn', @cancelWaitbar);

udata.cancelled = 0;
udata.timer = timer('StartDelay', 90, 'Tag', [get(wb, 'Tag') '_CancelButtonTimer'], 'TimerFcn', {@timerFcn_Callback, wb, get(wb, 'Tag')});%TO090806D
% set(udata.timer, 'StartFcn', @startFcn_Callback);
set(wb, 'UserData', udata);

return;

%------------------------------------------------------------
function cancelWaitbar(hObject, varargin)

udata = get(gcbf, 'UserData');
udata.cancelled = 1;

set(gcbf, 'UserData', udata);

try
    start(udata.timer);%TO090806D
catch
end

return;

%------------------------------------------------------------
%TO090806D
function startFcn_Callback(varargin)
return;

%------------------------------------------------------------
%TO090806D
function timerFcn_Callback(timerObj, event, wb, tag)

try
    if ishandle(wb)
        if strcmp(get(wb, 'Tag'), tag)
            try
                fprintf(1, 'Cancel command for waitbar ''%s'' (''%s'') appears to have left the figure in existence. Hiding figure...\n', ...
                    get(wb, 'Tag'), get(wb, 'Name'));
            catch
            end
            set(wb, 'Visible', 'Off');
        end
    end
catch
end

return;