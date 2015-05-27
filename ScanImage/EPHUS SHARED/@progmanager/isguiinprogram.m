function [out] = isguiinprogram(obj,prog_name,gui_name)
%ISGUIINPROGRAM   - @progmanager logical method for checking if GUI is in program.
%   ISGUIINPROGRAM method for progmanager to see if the gui name is added to program in program manager.
%   Must supply a progmanager object, a program name, and a gui name as input.
%
% See also ISSTARTED, ISPROGRAM

global progmanagerglobal
out=0;
if ~isprogram(obj,prog_name)
    warning(['isguiinprogram: program with name ' prog_name ' is not added to program manager']);
    return
else
    out=isfield(progmanagerglobal.programs.(prog_name).guinames,gui_name);
end
