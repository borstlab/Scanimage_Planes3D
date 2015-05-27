function out = getProgramLocal(obj, hObject, variableName, varargin)
% GETPROGRAMLOCAL - @progmanager method that gets a variable local to a specific program.
%
% SYNTAX
%     h = getProgramLocal(progmanager, hObject, variableName);
%
% ARGUMENTS
%     progmanager - The progmanager object.
%     hObject - The handle to a progmanagerized graphics object.
%     variableName - The name of the variable.
%
% NOTE
%     This does not handle multiple instances of the same variable name.
%     It will return the first instance it finds. Only use it for variables whose
%     name is unique to the entire program instance.
%            
% SEE ALSO GETLOCAL
%
% Created: Timothy O'Connor 6/10/04
%
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
global progmanagerglobal

out=[];

fighandle = getParent(hObject, 'figure');
uData = get(fighandle, 'UserData');

guiName = [];
guiNames = fieldnames(progmanagerglobal.programs.(uData.progname));
for i = 1 : length(guiNames)
    if strcmpi(class(progmanagerglobal.programs.(uData.progname).(guiNames{i})), 'struct') ...
            & ~ismember(guiNames{i}, {'mainGUIname', 'guinames', 'started', 'program_object', ...
                'program_object_filename', 'program_needs_saving'})
        if structHasField(progmanagerglobal.programs.(uData.progname).(guiNames{i}), 'variables')
            if structHasField(progmanagerglobal.programs.(uData.progname).(guiNames{i}).variables, variableName)
                guiName = guiNames{i};
                break;
            end
        end
    end
end

if isempty(guiName)
    error('Variable ''%s'' not found in program ''%s''.', variableName, uData.progname);
end

out = getglobal(obj, variableName, guiName, uData.progname, varargin{:});

return;