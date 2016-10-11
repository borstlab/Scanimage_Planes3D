function updateVariableFromGUI(obj,handle)
%UPDATEVARIABLEFROMGUI - Callback for GUI handling used by Program manager.
%   UPDATEVARIABLEFROMGUI(obj,handle) is placed in the 'Callback' property for any variables
%   tied to a GUI. BUILTINCALLBACK accepts as inputs the reference to the program manager object and 
%   the handle of the currently executed callback uicontrol object (gcbo).
%
%   UPDATEVARIABLEFROMGUI does the following:
%     1.  Verifies data in GUI.
%     2.  Forces value to conform to class specified.
%     3.  Updates variable associated with GUI.
%     4.  Updates other GUIs with the correct value.
%     5.  Optionally executes callbacks for other UICONTROLS as well.
%     6.  Sets flag to remember to save program on closing if a variable
%         in the save file was changed.
%
%   See also GETGUIVALUE, SETGUIVALUE,PROGMANAGER
global progmanagerglobal
fighandle=getParent(handle,'figure');
userdata=get(handle, 'UserData');   % UserData  of uicontrol
if isempty(userdata)
%     warning('@progmanager found an empty UserData field for handle %s.', get(handle, 'Tag'));
    return
end

value=getGUIValue(obj,handle);

if isfield(userdata,'variable')
    variablename=userdata.variable;
    [var_name,gui_name,program_name]=parseStructString(variablename);
    progmanagerglobal.programs.(program_name).(gui_name).variables.(var_name)=value;
    % Check the config flag for the variable.  If it is set to the
    % ConfigBitForSaving from the Program manager object, then we need to
    % remember we need to save this configuration.
    bit=getProgmanagerDefaults(obj,'ConfigBitForSaving');   % Bit to check for saving (little endian)
    flag=fliplr(dec2bin(getConfigFlag(obj,var_name,gui_name,program_name),2));  % Binary of configflag, reversed to make it same endian as bit index.
    if strcmp(flag(bit),'1')    %if the correct bit is 1, we should save this variable.
        setProgramProp(obj,program_name,'program_needs_saving',1); % Something changed so we need to remember this on close.
%         set(getMenuHandles(obj,program_name,'Tag','save'),'Enable','On');
    end
    otherguis=progmanagerglobal.programs.(program_name).(gui_name).variableGUIs.(var_name);
    for guicounter=1:length(otherguis)
        [obj_name,gui_name_gui,prog_name_gui]=parseStructString(otherguis{guicounter});
        setGUIValue(obj,progmanagerglobal.programs.(prog_name_gui).(gui_name_gui).guihandles.(obj_name),value);
    end

else
    warning('@progmanager could not find a linked variable in the UserData field for handle %s.', get(handle, 'Tag'));
    return;
end

return;