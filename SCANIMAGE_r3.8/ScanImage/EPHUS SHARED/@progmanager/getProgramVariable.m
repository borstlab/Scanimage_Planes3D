function variableValue = getProgramVariable(obj, programId, variableName)
% GETPROGRAMVARIABLE - @progmanager method that gets variables local to a specific program.
%
% SYNTAX
%     setProgramVariable(progmanager, programName, variableName);
%     setProgramVariable(progmanager, hObject, variableName);
%
% ARGUMENTS
%     progmanager - The progmanager object.
%     programName - The name of the program.
%     hObject - A progmanagerized graphics handle.
%     variableName - The variable whose value is to be retrieved.
%            
% SEE ALSO setLocal, setGlobal, setProgramVariable
%
% Created: Timothy O'Connor 6/11/04
%
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
global progmanagerglobal

if strcmpi(class(programId), 'char')
    programName = programId;
elseif ishandle(programId)
    uData = get(getParent(programId, 'figure'), 'UserData');
    if isempty(uData)
        error('@progmanager/setProgramVariable: user data not found in figure: %s', get(getParent(programId, 'figure'), 'Tag'));
    end

    programName = uData.progname;
else
    error('@progmanager/setProgramVariable: invalid program identifier.');
end

if ~isfield(progmanagerglobal.programs, programName)
    error('@progmanager/setProgramVariable: invalid program name - ''%s''.', programName);
end

if isempty(progmanagerglobal.programs.(programName).variables)
    error('@progmanager/setProgramVariable: invalid variable name, ''%s'', in program ''%s''.', variableName, programName);
end

if ~isfield(progmanagerglobal.programs.(programName).variables, variableName)
    error('@progmanager/setProgramVariable: program ''%s'' contains no variables, at the program level.', programName);
end

variableValue = progmanagerglobal.programs.(programName).variables.(variableName);

return;