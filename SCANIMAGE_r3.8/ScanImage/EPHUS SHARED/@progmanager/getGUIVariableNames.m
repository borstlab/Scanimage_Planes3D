function variableNames = getGUIVariableNames(obj, hObject)
%getGUIVariableNames - Returns a list (cell array of strings) of all 
%                      variables associated with this GUI.
%
%  SYNTAX
%    variableNames = getGUIVariables(progmanager, hObject);
global progmanagerglobal;

variableNames = {};

userdata = get(getParent(hObject, 'Figure'), 'UserData');

variableNames = sort(fieldnames(progmanagerglobal.programs.(userdata.progname).(userdata.guiname).variables));

return;