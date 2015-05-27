function out=hasUserDataField(handle, fieldname)
%HASUSERDATAFIELD   - Parses UserData of object as structure.
%   HASUSERDATAFIELD treats the UserData of the object specified by handle
%   as a structure array, and checks if the fieldname exists in the
%   structure.
%
% See also ISFIELD

% TPMOD_1 2/9/04: Commented.

ud=get(handle, 'UserData');
if isstruct(ud)
    out=isfield(ud, fieldname);
else
    out=0;
end

