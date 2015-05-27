function out = checkVersion(progmanager_obj, program_obj)
%CHECKVERSION   - progmanager private function for checking integrity of program file beign loaded to program manager.
%
% CHECKVERSION(progmanager_obj, program_obj) outputs 1 if version is
% correct, or 0 otherwise.

out=1;
[current_program_version,current_progmanager_version]=getProgramVersion(progmanager_obj,program_obj);
stored_program_version=get(program_obj,'version');
stored_progmanager_version=getProgmanagerDefaults(progmanager_obj,'version');

if ~isequal([current_program_version current_progmanager_version],[stored_program_version stored_progmanager_version])
    out=0;
end