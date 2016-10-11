% @progmanager/getUserDataHeaderValue - Retrieves a non-program-specific user data header value.
%
% SYNTAX
%  headerValue = getUserDataHeaderValue(pm, headerCategory, headerName)
%   pm - The program manager.
%   headerCategory - The category for this header. Just used as an organizational device.
%   headerName - The name of the header field, within the category, to be queried.
%   headerValue - The value to be retrieved from the header field.
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
function headerValue = getUserDataHeaderValue(this, headerCategory, headerName)
global progmanagerglobal;

if ~isfield(headers.userDataHeaders, headerCategory)
    error('No such header category: ''%s''', headerCategory);
end
if ~isfield(headers.userDataHeaders, headerName)
    error('No such header category: ''%s''', headerName);
end
headerValue = headers.userDataHeaders.(headerCategory).(headerName);

return;