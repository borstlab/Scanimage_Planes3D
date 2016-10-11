function type = structFieldExists(name, varargin)
% STRUCTFIELDEXISTS  - Determines if a specific field on an arbitrarily complex structure exists.
% 
% Created: Tim O'Connor 2/19/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
%
% SYNTAX
%     type = structFieldExists(name)
%     type = structFieldExists(name, PropertyName, PropertyValue, ...)
%     
% ARGUMENTS
%     NAME - A fully qualified ('.'-separated) structure name.
%     PROPERTYNAME - The name associated with a key-value pair.
%     PROPERTYVALUE - The value associated with a key-value pair.
%     
%     PropteryNames and PropertyValues MUST be specified in pairs. Any number of pairs may be specified, 
%     in any order, given that each PropteryName immediately preceeds a PropertyValue.
%     
% PROPERTIES
%     WORKSPACE - Specifies the workspace in which to look for the field. Any workspace, as defined for 
%                 `evalin` is valid. The default is 'caller'.
%     
% RETURNS
%     type - Returns a string if the field exists, 0 otherwise. The exact value returned is the class of 
%            the field's variable.
%     
% DESCRIPTION
%     bool = structFieldExists(name) - Returns a non-zero if, and only if, a field specified by the the 
%                                      fully qualified name exists.
%
%     bool = structFieldExists(name, 'workspace', 'base') - Returns a non-zero if, and only if, the fully 
%                                                           qualified name exists in the base workspace.
%     
% EXAMPLES
%     
%     structFieldExists('gh.montageGUI.montageGUI')
%     ans =
%           0
%
%     gh.montageGUI.montageGUI = figure;
%     structFieldExists('gh.montageGUI.montageGUI')
%     ans =
%           'double'
%
%     gh = 1;
%     structFieldExists('gh.montageGUI.montageGUI')
%     ans =
%           0
%
%  Changed:
%          3/6/04 Tim O'Connor (TO030604a): Allow it to work on multidimensional structures.
%          3/6/04 Tim O'Connor (TO030604b): Strip off multidimensional indices, if one exists
%                                           they all exist.
%           
% See Also CLASS, EVALIN, EXIST

%A little idiot-proofing.
if ~strcmpi(class(name), 'char')
    error('MATLAB:badopt', 'The variable name must be passed as a string.');
end

%Check number of arguments.
if mod(nargin - 1, 2) ~= 0
    error('MATLAB:badopt', 'All key-value pairs must consist of exactly 2 arguments.');
end

%Return value.
type = logical(0);

%Defaults.
workspace = 'caller';

%Process key-value pairs.
for i = 1 : 2 : nargin - 1
    if strcmpi(varargin{i}, 'WORKSPACE')
        workspace = varargin{i + 1};
    else
        error('MATLAB:badopt', sprintf('Invalid property specified: ''%s'' = %s', varargin{i}, varargin{i + 1}));
    end
end

%Parse out the pieces of the structure's name.
[root rem] = strtok(name, '.');
%Strip off multidimensional indices. --Tim O'Connor 3/6/04 TO30604b
[root countRem] = strtok(root, '(');

%Watch out for multidimensional structures that are too short.
count = strtok(countRem, ')');
if ~isempty(count)
    if length(root) < str2num(count)
        return;
    end
end

%Check to see if the structure exists at all.
if ~evalin(workspace, sprintf('exist(''%s'');', root))
    %Doesn't exist, game over.
    return;
end

%Make sure that it is a struct.
if ~evalin(workspace, sprintf('strcmpi(class(%s), ''struct'');', root))
    %The base variable exists, but it's not a struct, hence the field doesn't exist.
    return;
end

%Build up the full path as we go.
path = root;

%Loop through each field in the structure's name.
while ~isempty(rem)
    
    %Parse some more.
    [root rem] = strtok(rem, '.');
    %Strip off multidimensional indices. --Tim O'Connor 3/6/04 TO30604b
    [root countRem] = strtok(root, '(');

    %Watch out for multidimensional structures that are too short.
    count = strtok(countRem, ')');
    if ~isempty(count)
        if length(root) < str2num(count)
            return;
        end
    end
    
    %Make sure that it is a struct.
    if ~evalin(workspace, sprintf('strcmpi(class(%s), ''struct'');', path))
        %The base variable exists, but it's not a struct, hence the field doesn't exist.
        return;
    end
    
    %The sprintf evaluates to: ismember(fieldName, fieldnames(rootStruct))
    if ~evalin(workspace, sprintf('ismember(''%s'', fieldnames(%s));', root, path))
        %Nope, doesn't exist. Time to return 0.
        return;
    end
    
    %Keep building the path. By the end, assuming the field exists: strcmp(path, name) == 1
    %TO030604a, index into the parent node, in case it's multidimensional. -- Tim O'Connor 3/6/04
    path = strcat(path, '(1).', root);
        
    %Found it.
    if isempty(rem)
        %Now get the class.
        %TO030604a - Use path, instead of name, since path has indexing handled. -- Tim O'Connor 3/6/04
        type = evalin(workspace, sprintf('class(%s);', path));

        %Done.
        return;
    end
end

%Ebedebedebedebeh... that's all folks!
return;