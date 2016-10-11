function out = setProgramProp(prog_manager_obj,program_id,prop,value)
%SETPROGRAMPROP   -  @progmanager method for outputing the program level properties.
%   SETPROGRAMPROP(prog_manager_obj,program_id,prop,value) sets values for
%   the program.  Possible values that can be set are:
%
% 		program_object_filename [char]      filename where last program object copy was loaded.
%       program_needs_saving    [bool]      1 if program changed, 0 if not.
%
%   See also GETPROGRAMPROP
if nargin < 4
    error(['@progmanager/setProgramProp: too few inputs.']);
end
[prog_name]=parseProgramID(program_id);
global progmanagerglobal 
progmanagerglobal.programs.(prog_name).(prop)=value;
