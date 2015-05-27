function h = getFigHandle(OBJ, programID)
% Returns the main figure handle for a given program object.
%
% h = getFigHandle(OBJ, programID)
global progmanagerglobal;

[programName, programObj] = parseProgramID(programID);

h = progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).fighandle;

return;