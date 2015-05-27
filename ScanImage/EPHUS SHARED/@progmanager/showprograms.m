function [progam_names] = showprograms(obj)
%SHOWPROGRAMS   - @progmanager method for displaying the program names currently added.
%   SHOWPROGRAMS method for progmanager to see programs added to program manager.
%   Must supply a progmanager object as input.
%
% See also SHOWGUIS

global progmanagerglobal
progam_names=fieldnames(progmanagerglobal.programs);