function updateGUIsFromVariable(program_name,gui_name,variable_name,value)
% UPDATEGUISFROMVARIABLE   - private function for @progmanager class.
%   UPDATEGUISFROMVARIABLE sets the GUIs that are tied to a variable specified in variable_name
%   to that value.
%
%   See also UPDATEVARIABLEFROMGUI, SETGLOBAL

global progmanagerglobal
% Are there GUIs connected to this variable? If so, we better update them.
if isfield(progmanagerglobal.programs.(program_name).(gui_name).variableGUIs,(variable_name))
    otherguis=progmanagerglobal.programs.(program_name).(gui_name).variableGUIs.(variable_name);
    for guicounter=1:length(otherguis)
        [obj_name,gui_name_gui,prog_name_gui]=parseStructString(otherguis{guicounter});
        setGUIValue(progmanager,progmanagerglobal.programs.(prog_name_gui).(gui_name_gui).guihandles.(obj_name),...
            progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name));
    end
end
