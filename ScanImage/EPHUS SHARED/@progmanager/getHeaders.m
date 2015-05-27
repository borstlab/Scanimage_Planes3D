% PROGMANAGER/getHeaders
%
% SYNTAX
%  headers = getHeaders(progmanager)
%  headers = getHeaders(progmanager, hObjects)
%   progmanager - Program Manager.
%   hObject - Program handles for all programs from which to retrieve header data (if not specified, all running programs are accessed).
%   headers - A structure containing fields for all programs, with subfields for each gui, with subfields for each variable flagged as a header variable.
%
% USAGE
%
% NOTES
%  This is a copy & paste job from `getProgramSettings` with changes as necessary.
%
%  For now, there is no `genericPostGetHeader` function called. This may, however, be implemented at some time in the future
%  so all programs must support it. - 11/30/05
%
% CHANGES
%  TO112805B: Use `bitand` when checking config flags. -- Tim O'Connor 11/28/05
%  TO060208J - Added a non-program oriented header, for assorted user data. -- Tim O'Connor 6/1/08
%
% Created 11/30/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function headers = getHeaders(this, varargin)
global progmanagerglobal;

headers = [];
if ~isempty(varargin)
    hObjects = varargin{1};
else
    programs = fieldnames(progmanagerglobal.programs);
    for i = 1 : length(programs)
        hObjects(i) = progmanagerglobal.programs.(programs{i}).guinames.(progmanagerglobal.programs.(programs{i}).mainGUIname).fighandle;
    end
    headers.userDataHeaders = progmanagerglobal.internal.userDataHeaders;%TO060208J
end

if ~isfield(progmanagerglobal.internal, 'ephusVersion')
    versionFile = fopen('ephus_version.tag', 'r');
    progmanagerglobal.internal.ephusVersion = fgetl(versionFile);
    fclose(versionFile);
end

for h = 1 : length(hObjects)
    hObject = hObjects(h);
    [programName, programObj] = parseProgramID(hObject);
    guinames = fieldnames(progmanagerglobal.programs.(programName).guinames);
    headers.ephusVersion = progmanagerglobal.internal.ephusVersion;

    for i = 1 : length(guinames)
        try
            feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, 'genericPreGetHeader', ...
                progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
                progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
        catch
            warning('Encountered a malfunctioning genericPreGetHeader function for %s:%s - %s', programName, guinames{i}, getLastErrorStack);
        end
        
        version = feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, 'getVersion', ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
        headers.(programName).(guinames{i}).version = version;

        variableNames = fieldnames(progmanagerglobal.programs.(programName).(guinames{i}).configflags);        
        for j = 1 : length(variableNames)
            if bitand(progmanagerglobal.programs.(programName).(guinames{i}).configflags.(variableNames{j}), 2) %TO112805B
                headers.(programName).(guinames{i}).(variableNames{j}) = progmanagerglobal.programs.(programName).(guinames{i}).variables.(variableNames{j});
            end
        end
        
        try
            feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, 'genericPostGetHeader', ...
                progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
                progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
        catch
            warning('Encountered a malfunctioning genericPostGetHeader function for %s:%s - %s', programName, guinames{i}, getLastErrorStack);
        end
    end
end
    
return;