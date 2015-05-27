% texSafe - Alter a string, to make it TeX safe.
%
% SYNTAX
%   texSafeString = texSafe(string)
%
% USAGE
%
% NOTES
%   This is will insert escapes, where necessary.
%
% CHANGES
%
% Created 2/16/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function string = texSafe(string)

%Make it TeX safe.
string = strrep(string, '\', '\\');
string = strrep(string, '_', '\_');

return;