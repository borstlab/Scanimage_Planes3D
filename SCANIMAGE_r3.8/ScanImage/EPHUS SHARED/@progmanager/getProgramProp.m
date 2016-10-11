function out = getProgramProp(prog_manager_obj,program_id,prop)
%GETPROGRAMPROP   -  @progmanager method for outputing the program level properties.
%   GETPROGRAMPROP(prog_manager_obj,program_id) returns the entire structure of the
%   program from the program manager global strucutre.
%     
% 	GETPROGRAMPROP(prog_manager_obj,program_id,prop) returns the value of the
%   prop specified based on the program_id.
%
%   Possible prop values are:
% 		mainGUIname             [char]      name of the main gui for this program.
% 		guinames                [struct]    structure containing the guinames as fields
%                                           and the m_filename, fighandle, and funchandle
%                                           as subfields.
% 		started                 [bool]      1 is started, 0 if not.
% 		program_object          [program]   program object copy.
% 		program_object_filename [char]      filename where last program object copy was loaded.
%
%   See also SHOWGUIS, SHOWPROGRAMS, GETGUINAME, PARSEPROGRAMID
if nargin < 2
    error(['@progmanager/getProgramProp: too few inputs.']);
end
[prog_name]=parseProgramID(program_id);
global progmanagerglobal 
if nargin == 2
    out=progmanagerglobal.programs.(prog_name);
else
    out=progmanagerglobal.programs.(prog_name).(prop);
end