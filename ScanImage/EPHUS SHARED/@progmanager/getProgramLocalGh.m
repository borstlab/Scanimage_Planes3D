function out = getProgramLocalGh(obj, hObject, guiObjectName, varargin)
% GETPROGRAMLOCALGH - @progmanager method that gets a GUI handle local to a specific program.
%
% SYNTAX
%     h = getProgramLocal(progmanager, hObject, variableName);
%
% ARGUMENTS
%     progmanager - The progmanager object.
%     hObject - The handle to a progmanagerized graphics object.
%     guiObjectName - The GUI object's handle.
%
% NOTE
%     This does not handle multiple instances of the same GUI handle name.
%     It will return the first instance it finds. Only use it for GUI handle whose
%     name is unique to the entire program instance.
%            
% SEE ALSO GETLOCALGH
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
        if structHasField(progmanagerglobal.programs.(uData.progname).(guiNames{i}), 'guihandles')
            if structHasField(progmanagerglobal.programs.(uData.progname).(guiNames{i}).guihandles, guiObjectName)
                guiName = guiNames{i};
                break;
            end
        end
    end
end

if isempty(guiName)
    error('GUI ''%s'' not found in program ''%s''.', guiObjectName, uData.progname);
end

out = getglobal(obj, guiObjectName, guiName, uData.progname, varargin{:});

return;