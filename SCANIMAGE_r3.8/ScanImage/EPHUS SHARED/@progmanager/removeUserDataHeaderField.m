% @progmanager/removeUserDataHeaderField - Removes a non-program-specific user data header field.
%
% SYNTAX
%  removeUserDataHeaderField(pm, headerCategory, headerName)
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
function removeUserDataHeaderField(this, headerCategory, headerName)
global progmanagerglobal;

if ~isfield(progmanagerglobal.internal.userDataHeaders, headerCategory)
    warning('No such header category: ''%s''', headerCategory);
end
if ~isfield(progmanagerglobal.internal.userDataHeaders, headerName)
    warning('No such header category: ''%s''', headerName);
end

progmanagerglobal.internal.userDataHeaders = rmfield(progmanagerglobal.internal.userDataHeaders.(headerCategory), headerName);

return;