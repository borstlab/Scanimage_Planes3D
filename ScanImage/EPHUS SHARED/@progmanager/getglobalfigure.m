function out = getglobalfigure(prog_object, program_name, gui_name)
% GETGLOBALFIGURE   - @progmanager method that gets handle to GUI object for the named figure.
%
% SYNTAX
%     h = getglobalfigure(progmanager, programName, guiName);
%
% ARGUMENTS
%     progmanager - The progmanager object.
%     programName - The name of the program.
%     guiName - The name of the GUI.
%            
% SEE ALSO GETGLOBALGH
%
% Created: Timothy O'Connor 6/10/04
%
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
global progmanagerglobal

if isfield(progmanagerglobal.programs.(program_name), gui_name)
    out = progmanagerglobal.programs.(program_name).(gui_name).guihandles.(lower(gui_name));
else
    error('@progmanager/getglobalgh: invalid GUI name ,''%s'', in program ''%s''.', gui_name, program_name);
end