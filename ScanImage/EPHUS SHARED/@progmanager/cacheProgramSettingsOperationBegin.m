% PROGMANAGER/cacheProgramSettingsOperationBegin
%
% SYNTAX
%  cacheProgramSettingsOperationBegin(progmanager, hObject)
%   progmanager - Program Manager.
%   hObject - Program handle.
%
% USAGE
%  This should be called before caching of configurations has begun, in order for programs to prepare to cache a series of 
%  configurations.
%
% NOTES
%  See TO062806C.
%
% CHANGES
%
% Created 6/28/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function cacheProgramSettingsOperationBegin(this, hObject)
global progmanagerglobal;

[programName, programObj] = parseProgramID(hObject);

guinames = fieldnames(progmanagerglobal.programs.(programName).guinames);

for i = 1 : length(guinames)
    try
        feval(progmanagerglobal.programs.(programName).guinames.(guinames{i}).funchandle, 'genericCacheOperationBegin', ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle, [], ...
            progmanagerglobal.programs.(programName).guinames.(guinames{i}).fighandle);
    catch
        warning('Encountered a malfunctioning genericCacheOperationComplete function for %s:%s - %s', programName, guinames{i}, lasterr);
    end
end

return;