function boolean = structHasField(struct, varargin)
% STRUCTHASFIELD - Checks to see if a list of fields are parts of a specified structure.
%
% SYNTAX
%     boolean = structHasField(struct, fieldName, ...)
%
% ARGUMENTS
%     struct - The structure to be analyzed.
%     fieldName - The field to check for existence on the structure. Multiple fieldnames may be checked at once.
%
% RETURNS
%     A boolean, 1 if the field exists, 0 otherwise. One value is returned for each field queried.
%
% NOTE
%     This only looks one level of depth into the structure. For deeper searching use STRUCTFIELDEXISTS.
%     Also, for large structures, STRUCTFIELDEXISTS should provide better performance.
%
% EXAMPLES
%     a.a = 1;
%     a.b = 2;
%     structHasField(a, 'a')
%     ans =
%           1
%     structHasField(a, 'c')
%     ans =
%           0
%     structHasField(a, 'a', 'b', 'c')
%     ans =
%           [1 1 0]
%
% SEE ALSO STRUCTFIELDEXISTS, EXIST
%
% CREATED
%     Timothy O'Connor 6/8/04
%     Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
if ~strcmpi(class(struct), 'struct')
    error('MATLAB:badopt', 'The first argument to structHasField must be a structure: %s', class(struct));
end

boolean = zeros(nargin - 1, 1);

fnames = fieldnames(struct);
for i = 1 : nargin - 1
    boolean(i) = ismember(varargin{i}, fnames);
end

return;