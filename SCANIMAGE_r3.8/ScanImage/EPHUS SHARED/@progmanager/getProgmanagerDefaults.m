function out=getProgmanagerDefaults(prog_object,property)
% GETPROGMANAGERDEFAULTS   - @progmanager method returns the default settings for the program manager.
%   GETPROGMANAGERDEFAULTS(prog_object,property) gets the value specified property from the
%   program manager global array.  
% 
%   GETPROGMANAGERDEFAULTS(prog_object) returns the entire intenral
%   structure of the current program manager.
%
%   See PROGMANAGER for details on the default properties that can be set.
%
%   See also SETPROGMANAGERDEFAULTS, PROGMANAGER.

if nargin ==2
    out=evalin('base',[prog_object.name '.internal.' property]);
elseif nargin ==1
    out=evalin('base',[prog_object.name '.internal']);
end
