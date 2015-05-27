function toggleGuiVisibility(this, hObject, guiName, varargin)
% Sets the named GUI's visibility and toggles the checkmark in the main GUI's menu.
%
% Syntax 
%  toggleGuiVisibility(progmanager, hObject, guiName)
%  toggleGuiVisibility(progmanager, hObject, guiName, 'On') - Force it to 'On'
%  toggleGuiVisibility(progmanager, hObject, guiName, 'On') - Force it to 'Off'
%
% Changes
%  TO093005A: Completely reworked menus to be much more useful. Made a programmanger submenu on file. -- Tim O'Connor 9/30/05
%
% Created - Tim O'Connor 6/16/04
%
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
global progmanagerglobal

udata = get(getParent(hObject, 'figure'), 'UserData');

if isfield(progmanagerglobal.programs.(udata.progname).guinames, guiName)
    if isfield(progmanagerglobal.programs.(udata.progname).guinames.(guiName), 'fighandle')
        if ~isempty(varargin)
            newState = varargin{1};
        elseif strcmpi(get(progmanagerglobal.programs.(udata.progname).guinames.(guiName).fighandle, 'Visible'), 'On')
            newState = 'Off';
        else
            newState = 'On';
        end
        
        %Show/hide the GUI.
        set(progmanagerglobal.programs.(udata.progname).guinames.(guiName).fighandle, 'Visible', newState);
        
        %Update the menu item's check mark.
        menuItem = getMenuItem(progmanagerglobal.programs.(udata.progname).guinames.(progmanagerglobal.programs.(udata.progname).mainGUIname).fighandle, ...
            'Show GUI', guiName);
        set(menuItem, 'Checked', newState);
    else
        error('@progmanager/toggleGuiVisibility: no figure handle found for GUI ''%s'' in program ''%s''.', guiName, udata.progname);
    end
else
    error('@progmanager/toggleGuiVisibility: invalid GUI name ''%s'' in program ''%s''.', guiName, udata.progname);
end

%TO093005A
setWindowsMenuItems(this, 'toggle');

return;