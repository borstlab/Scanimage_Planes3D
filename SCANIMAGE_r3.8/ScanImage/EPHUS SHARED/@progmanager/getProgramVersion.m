function [program_manager_version,program_object_version] = getProgramVersion(prog_manager_obj,program_id)
%GETPROGRAMVERSION   -  @progmanager method for outputing the program manager and program object versions.
%   GETPROGRAMVERSION(prog_manager_obj,program_id) returns program manager
%   version and the current program version.  The current program version
%   is derioved from the m_file of the main GUI of the program.  The
%   program and program manager version are saved with the program object.
%
%   The program id can be a handle in the program, a copy of the program
%   object, or the name of the program.
%
%   See also 

program_manager_version=getProgmanagerDefaults(prog_manager_obj,'version');
program_object_version=NaN;

[prog_name]=parseProgramID(program_id);

global progmanagerglobal
main_gui_name=progmanagerglobal.programs.(prog_name).mainGUIname;
fig_handle=getHandleFromName(prog_manager_obj,main_gui_name,prog_name);
try
    program_object_version=feval(progmanagerglobal.programs.(prog_name).guinames.(main_gui_name).funchandle,'getVersion',fig_handle,[],fig_handle);    
end

