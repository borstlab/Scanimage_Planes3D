% @progmanager/deleteUserDataHeaders - Removes all non-program-specific user data header information.
%
% SYNTAX
%  removeUserDataHeaders(pm, headerCategory, headerName)
%   pm - The program manager.
%   headerCategory - The category for this header. Just used as an organizational device.
%   headerName - The name of the header field, within the category, to be deleted.
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
function removeUserDataHeaders(this, headerCategory)
global progmanagerglobal;

if ~isfield(progmanagerglobal.internal.userDataHeaders, headerCategory)
    warning('No such header category: ''%s''', headerCategory);
end

progmanagerglobal.internal.userDataHeaders = [];

return;