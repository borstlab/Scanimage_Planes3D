% PROGMANAGER/cacheProgramSettingsOperationComplete
%
% SYNTAX
%  cacheProgramSettingsOperationComplete(progmanager, hObject)
%   progmanager - Program Manager.
%   hObject - Program handle.
%
% USAGE
%  This should be called after all caching of configurations is complete, in order for programs to execute time consuming processes
%  that were not necessary while loading the individual settings.
%
% NOTES
%  See TO062806C.
%
% CHANGES
%
% Created 6/28/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function cacheProgramSettingsOperationComplete(this, hObject)
global progmanagerglobal;

[programName, programObj] = parseProgramID(hObject);

guinames = fieldnames(progmanagerglobal.programs.(programName).guinames);

for i = 1 : length(guinames)
    try
        feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, 'genericCacheOperationComplete', ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
    catch
        warning('Encountered a malfunctioning genericCacheOperationComplete function for %s:%s - %s', programName, guinames{i}, lasterr);
    end
end

return;