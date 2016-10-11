function out=getConfigFlag(obj,variable_name,gui_name,program_name)
% GETCONFIGFLAG   - @progmanager method that gets value of a variable's Config Flag.
%   GETCONFIGFLAG gets the value of the variable's config flag.  If none is
%   available, it return a flag of 0.
%
%   See also UPDATEVARIABLEFROMGUI

if nargin == 4
    global progmanagerglobal
    out=0;
    if isfield(progmanagerglobal.programs.(program_name).(gui_name).configflags,variable_name)
        out=progmanagerglobal.programs.(program_name).(gui_name).configflags.(variable_name);
    end
else
    error('@progmanager/getConfigFlag: must supply 4 inputs.  See help for details.');
end
