% PROGMANAGER/getLocalBatch - Used to set multiple local variables simultaneously.
%
% SYNTAX
%  [VARIABLE_VALUE, ...] = setLocalBatch(this, hObject, VARIABLE_NAME, ...)
%   this - @PROGMANAGER
%   hObject - The handle to the program.
%   VARIABLE_NAME - The name(s) of the variable to be get.
%   VARIABLE_VALUE - The value the named variable(s).
%
% USAGE
%  Specifying multiple name/value pairs at once allows much of the overhead in getting to be handled only once.
%  This optimization is ideal for the beginning of a function where a number of variables are retrieved.
%
% NOTES
%
% CHANGES
%  TO121305C - Made an error message a bit more verbose (added a stack trace). -- Tim O'Connor 12/13/05
%  TO122205A - Upgrade to Matlab 7.1, case sensitivity. -- Tim O'Connor 12/22/05
%
% Created 7/8/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function varargout = getLocalBatch(this, hObject, varargin)
global progmanagerglobal;

if strcmpi(class(hObject), 'program') %TO122205A
    program_name = get(hObject, 'program_name');
    gui_name = get(hObject, 'main_gui_name');
    hObject = progmanagerglobal.programs.(program_name).(gui_name).guihandles.(lower(program_name));
elseif ishandle(hObject)
    fighandle = getParent(hObject, 'figure');
    userData = get(fighandle,'UserData');
    gui_name = userData.guiname;
    program_name = userData.progname;
else
class(hObject)
hObject
getStackTraceString
    error('Invalid program/gui handle.');
end

if length(varargin) ~= nargout
    error('Incorrect number of output arguments. Expected %s. Found %s.\n%s\n', num2str(length(varargin)), num2str(nargout), getStackTraceString);%TO121305C
end

for i = 1 : length(varargin)
    varargout{i} = progmanagerglobal.programs.(program_name).(gui_name).variables.(varargin{i});
end

return;