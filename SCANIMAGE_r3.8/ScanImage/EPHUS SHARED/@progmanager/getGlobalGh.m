function out=getglobalgh(prog_object,handle_tag,gui_name,program_name)
% GETGLOBALGH   - @progmanager method that gets handle to GUI object with name TAG handle_tag.
%   GETGLOBALGH gets handle to GUI object with name TAG handle_tag.  This
%   is useful for setting properties of the various GUI objects when they
%   are not tied to a variable (like an axes handle).
%            
%   See also GETGLOBAL
%
%   TO122804a: Make program and gui names case insensitive for set/get purposes. -- Tim O'Connor 12/28/04

global progmanagerglobal
if nargin >= 4
    %TO122804a - Start
    programNames = fieldnames(progmanagerglobal.programs);
    programMatches = find(strcmpi(program_name, programNames));
    if length(programMatches) > 1
        error('@progmanager/getglobalgh: ambiguous program structures found for ''%s''', program_name);
    end
    if ~isempty(programMatches)
        program_name = programNames{programMatches};
    end
    
    guiNames = fieldnames(progmanagerglobal.programs.(program_name));
    guiMatches = find(strcmpi(gui_name, guiNames));
    if length(programMatches) > 1
        error('@progmanager/getglobalgh: ambiguous program structures found for GUI ''%s'' in program ''%s''', gui_name, program_name);
    end
    if ~isempty(programMatches)
        gui_name = guiNames{guiMatches};
    end
    
    if isempty(programMatches) | isempty(guiMatches)
        error(['@progmanager/getglobalgh: invalid handle tag ' handle_tag ' for GUI ' gui_name ' in program ' program_name]);
    end
    %TO122804a - End

    if isfield(progmanagerglobal.programs.(program_name).(gui_name).guihandles,handle_tag)
        out=progmanagerglobal.programs.(program_name).(gui_name).guihandles.(handle_tag);
    else
        error(['@progmanager/getglobalgh: invalid handle tag ' handle_tag ' for GUI ' gui_name ' in program ' program_name]);
    end
else
    error('@progmanager/getglobalgh: must supply 4 inputs.  See help for details.');
end
