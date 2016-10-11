function fighandle = getHandleFromName(prog_manager_obj,gui_name,program_name)
%GETHANDLEFROMNAME   -  @progmanager method for outputing the GUI figure handle from GUI instance's gui_name.
%   GETHANDLEFROMNAME(prog_manager_obj,gui_name,program_name) returns the
%   handle to the GUI with name gui_name(or alias if multiple copies exist) from program program_name.
%
%   This is very useful for setting variables in GUI subfunctions through
%   GETLOCAL and SETLOCAL, which take the figure handle of the GUI object
%   as inputs.  
%
%   See also GETLOCAL, SETLOCAL
global progmanagerglobal
fighandle=progmanagerglobal.programs.(program_name).guinames.(gui_name).fighandle;
