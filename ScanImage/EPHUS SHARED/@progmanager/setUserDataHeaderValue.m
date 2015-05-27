% @progmanager/setUserDataHeaderValue - Sets (creates, if necessary) a non-program-specific user data header value.
%
% SYNTAX
%  setUserDataHeaderValue(pm, headerCategory, headerName, headerValue)
%   pm - The program manager.
%   headerCategory - The category for this header. Just used as an organizational device.
%                    If it does not exist, it will be created.
%   headerName - The name of the header field, within the category, to be updated.
%                If it does not exist, it will be created.
%   headerValue - The value to be assigned to the header field.
%
% NOTES
%  See TO060208J.
%
% CHANGES
%
% Created
%  Timothy O'Connor 6/1/08
%
% Copyright
%  Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2008
function setUserDataHeaderValue(this, headerCategory, headerName, headerValue)
global progmanagerglobal;

if ~ischar(headerCategory)
    error('User data header categories must be strings not of type ''%s''.', class(headerCategory));
end
if ~ischar(headerName)
    error('User data header names must be strings not of type ''%s''.', class(headerName));
end

progmanagerglobal.internal.userDataHeaders.(headerCategory).(headerName) = headerValue;

return;