function [gui_names] = showGUIs(obj,program) %VI053108B
%SHOWGUIS   - @progmanager method for displaying the GUI names associated with a particular program 
% USAGE
%   gui_names = showGUIs(obj,program)
%       obj: @progmanager object
%       program: a string, specifying name of 
%
% See also SHOWPROGRAMS
% MODIFICATIONS
%   VI053108A: Show guinames fields, not program fields -- Vijay Iyer 5/31/08
%   VI053108B: Renamed to showGUIS (more symmetric with showPrograms) -- Vijay Iyer 5/31/08
global progmanagerglobal
gui_names=fieldnames(progmanagerglobal.programs.(program).guinames); %VI053108A