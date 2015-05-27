function bool = isFileHandle(fid)
% ISFILEHANDLE - Determines if a variable is a handle to an open file, or if it is
%                indeed a handle.
%
% Created: Timothy O'Connor 4/08/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
% 
% USAGE
%     bool = isFileHandle(fid)
%     
% ARGUMENTS
%     fid - The variable to be tested.
%
% RETURNS
%     bool - A logical, 1 if fid is a valid file handle, 0 otherwise.
%     
% See Also FOPEN
bool = logical(1);

if ~isnumeric(fid)
    bool = logical(0);
    return;
elseif fid == 1 | fid == 2
    return;
end

%Not the prettiest way... is there anything better?
try
    ftell(fid);
catch
    bool = logical(0);
end

return;