function out = isprogram(obj,program)
%ISPROGRAM   - @progmanager logical method for checking program name.
%   ISPROGRAM method for progmanager to see if program is added to program manager.
%   Must supply a progmanager object as input.
%
% TO092805H: Made case insensitive. Also, note that the argument taken is a string (the program's name), 
%            since the documentation here is effectively useless. -- Tim O'Connor 9/28/05
%
% See also ISSTARTED, ISGUIINPROGRAM

global progmanagerglobal
% out=isfield(progmanagerglobal.programs, program);
out = any(strcmpi(fieldnames(progmanagerglobal.programs), program));%TO092805H