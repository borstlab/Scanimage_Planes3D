function out=getVarWithConfigFlag(obj,program_name,gui_name,config_flag)
% GETVARWITHCONFIGFLAG   - @progmanager method that gets varibles with a specified config flag.
%   GETVARWITHCONFIGFLAG returns a structure of variables (fieldnames) and
%   values of those variables if they have the specified config_flag.  
%   
%   config_flag can be an array of possible config flags.
%
%   Example:  
% 		getVarWithConfigFlag(progmanager,'Program_Name','GUI_Name',[1 2])
%
%       will return all the variables stored for the 'Program_Name' in the
%       'GUI_Name' with ConfigFlags either 1 or 2.
%
%   This function is used when parsing the state of the program for saving
%   or outputting to a header.
%
%   See also PROGMANAGER

if nargin == 4
    global progmanagerglobal
    out=[];
    if isstruct(progmanagerglobal.programs.(program_name).(gui_name).configflags)
        varnames=fieldnames(progmanagerglobal.programs.(program_name).(gui_name).configflags);    % get all variables with config flags.
        bit=2.^(config_flag-1);
        for varCounter=1:length(varnames) % write the structure fields.
            if any(bitand(progmanagerglobal.programs.(program_name).(gui_name).configflags.(varnames{varCounter}),bit) == bit)
                out.(varnames{varCounter})=progmanagerglobal.programs.(program_name).(gui_name).variables.(varnames{varCounter});
            end
        end
    end
else
    error('@progmanager/getVarWithConfigFlag: must supply 4 inputs.  See help for details.');
end
