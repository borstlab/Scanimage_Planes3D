% setSvobodaLabPaths - Sets up the path for use with packaged software.
%
% SYNTAX
%  setSvobodaLabPaths
%  setSvobodaLabPaths(root)
%   root - The base directory, in which to find library, Programs, and USERCLASSES.
%
% NOTE
%  If no root is specified, a global variable called 'setSvobodaLabPaths_rootDir' is used. If 'setSvobodaLabPaths_rootDir' 
%  is undefined the root path gets defaulted to 'C:\Matlab6p5\work\svobodalab'. If that does not exist the user 
%  is prompted to choose a directory.
%
% Created 8/23/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function setSvobodaLabPaths(varargin)
global setSvobodaLabPaths_rootDir;

if length(varargin) == 1
    root = varargin{1};
elseif ~isempty(setSvobodaLabPaths_rootDir)
    root = setSvobodaLabPaths_rootDir;
else
    root = 'C:\Matlab6p5\work\svobodalab';
end

if exist(root) ~= 7
    root = uigetdir(matlabroot, 'Choose the directory into which you have installed the software packages.');
    if isempty(root)
        return;
    end
    if length(root) == 1
        if root == 0
            return;
        end
    end
end

lib = [root '\library'];
progs = [root '\Programs'];
classes = [root '\USERCLASSES'];

%Hang onto this, because we'll want to check if things are already on the path, so we don't have redundant entries.
%If redundancy is allowed, multiple calls to this function would result in a massive path.
p = lower(path);

addRecursive(lib, p);
addRecursive(progs, p);
addDir(classes, p);

yesOrNo = questdlg('The path must now be saved for use in future sessions. Would you like to do this now?', ...
    'Save Path?', 'Yes', 'No', 'Yes');
if strcmpi(yesOrNo, 'Yes')
    path2rc;
end

return;

%----------------------------------------------
function addDir(name, p)

if exist(name) ~= 7
    fprintf(2, 'WARNING - Expected directory not found: %s\n', name);
    return;
end

fprintf(1, 'Adding ''%s'' to the path.\n', name);
if ~isempty(strfind(p, lower(name)))
    %Remove here, then prepend later, so it takes precedence in the search path.
    rmpath(name);
end
addpath(name);

return;

%----------------------------------------------
function addRecursive(name, p)

if exist(name) ~= 7
    fprintf(2, 'WARNING - Expected directory not found: %s\n', name);
    return;
end

addDir(name, p);

d = dir(name);
for i = 1 : length(d)
    if d(i).isdir & ~strcmp(d(i).name, '.') & ~strcmp(d(i).name, '..')
        addRecursive(fullfile(name, d(i).name), p);
    end
end

return;