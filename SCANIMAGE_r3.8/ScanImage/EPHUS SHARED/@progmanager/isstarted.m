function [out] = isstarted(obj,program_id)
%ISSTARTED   - @progmanager logical method for checking program name.
%   ISSTARTED method for progmanager to see if program is started.
%   Must supply a progmanager object as input.
%
% SYNTAX
%  started = isstarted(progmanager, program_id)
%   started - 1 if the program is running, 0 otherwise.
%   progmanager - The progmanager instance.
%   program_id - A string, figure handle, or @program instance.
%
% See also ISPROGRAM
%
% Changed:
%         6/10/04 Tim O'Connor - The call to warning was seriously borked. And, is possibly unwanted. - TO061004a

[prog_name]=parseProgramID(program_id);
if ~isprogram(obj,prog_name)
    %TO061004a - Is this really necessary? Does a call to this need to be wrapped by `isprogram` or
    % should this be sufficient without a warning?
%     warning('progmanager/isstarted: program with name ''%s'' is not added to program manager.', program_id);
    out=0;
    return
else
    global progmanagerglobal
    out=progmanagerglobal.programs.(prog_name).started;
end