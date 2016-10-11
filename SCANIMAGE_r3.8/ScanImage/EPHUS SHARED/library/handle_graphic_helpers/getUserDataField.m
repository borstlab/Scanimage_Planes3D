function retVal=getUserDataField(handle, field)
%GETUSERDATAFIELD   - Parses UserData of object as structure.
%   GETUSERDATAFIELD treats the UserData of the object specified by handle
%   as a structure array, and checks if the fieldname exists in the
%   structure, and if so, returns the value of that field.
%
%   See also ISFIELD, HASUSERDATAFIELD

%   TPMOD_1 2/9/04: Commented and made tro use dynamic fieldnames.
if (hasUserDataField(handle, field))
    ud=get(handle, 'UserData');
    retVal=ud.(field);
else
    retVal=[];
end
	