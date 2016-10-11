% PROGMANAGER/setGlobalGhBatch - Set a "benign" field across multiple GUI objects, simultaneously.
%
% SYNTAX
%  setGlobalGhBatch(progmanager, programName, guiName, tags, propertyName, propertyValue, ...)
%    progmanager - The program manager object.
%    programName - The name of the program in which the GUI exists.
%    guiName - The name of the gui in which the elements exist.
%    tag - The tag of the element to be set. A cell array may be provided to set multiple elements at once.
%    propertyName - The property to be set (may be a cell array).
%                   Multiple pairs of propertyName and propertyValue may be used.
%    propertyValue - The value to which to set the property (if propertyName is a cell array, value must be a cell array of the same length).
%                   Multiple pairs of propertyName and propertyValue may be used.
%
% NOTE
%  This function is intended, specifically, for quickly setting the Enable and Visible properties, which have no effect on
%  the value of any linked variables. Other possible uses include setting the Position, colors, etc. No efficiency will be gained
%  by using this to set properties that are linked to variable values (Value, String, Min, Max). It may, in fact, be less
%  efficient to set such properties with this function. A warning will be issued if any such attempt is made.
%
%  See TO110705A.
%
% CHANGES
%
% Created 11/7/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function setGlobalGhBatch(this, programName, guiName, tags, varargin)
global progmanagerglobal;

if isempty(tags)
    error('No tags specified.');
end

if mod(length(varargin), 2) ~= 0
    error('Argument mismatch - propertyNamess and propertyValues must be specified in pairs.');
end

propertyNames = {varargin{1 : 2 : end - 1}};
propertyValues = {varargin{2 : 2 : end}};

illegalProperties = intersect(lower(propertyNames), {'string', 'value', 'min', 'max'});
if ~isempty(illegalProperties)
    s = illegalProperties{1};
    for i = 2 : length(illegalProperties)
        s = [s ', ' illegalProperties{i}];
    end
    warning('It is not recommended to set value-influencing properties via setGlobalGhBatch: %s\nIterating calls to setGlobalGh (may be slower).', s);
end

if strcmpi(class(tags), 'char')
    tags = {tags};
end

missingHandles = {};
for i = 1 : length(tags)
    try
        handles(i) = progmanagerglobal.programs.(programName).(guiName).guihandles.(tags{i});
    catch
        missingHandles{length(missingHandles) + 1} = tags{i};
    end
end
if ~isempty(missingHandles)
    if length(missingHandles) == 1
        error('Failed to access handle for ''%s:%s:%s'' - %s', programName, guiName, missingHandles{1}, lasterr);
    else
        s = sprintf('%s:%s:%s', programName, guiName, missingHandles{i});
        for i = 2 : length(missingHandles)
            s = sprintf('%s\n  ''%s:%s:%s''', s, programName, guiName, missingHandles{i});
        end
        error('Failed to access handles for:\n  %s\n%s', lasterr);
    end
end

for i = 1 : length(propertyNames)
    if ~ismember(lower(propertyNames{i}), {'string', 'value', 'min', 'max'})
        set(handles, propertyNames{i}, propertyValues{i});
    else
        for j = 1 : length(tags)
            setGlobalGh(this, tags{j}, guiName, programName, propertyNames{i}, propertyValues{i});
        end
    end
end

return;