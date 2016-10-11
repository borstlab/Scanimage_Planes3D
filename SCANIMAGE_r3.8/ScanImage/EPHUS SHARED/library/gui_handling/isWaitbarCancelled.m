function cancelled = isWaitbarCancelled(wb)
% waitbarWithCancel - Returns 1 if the waitbar has been cancelled
% 0 otherwise.
%
% CHANGES
%  TO053108D - Cleaned up to better handle error cases and deletion.
%
% Created: Tim O'Connor 6/11/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004

%TO053108D
if ~ishandle(wb)
    cancelled = 1;
    return;
end

udata = get(wb, 'UserData');

cancelled = udata.cancelled;

return;