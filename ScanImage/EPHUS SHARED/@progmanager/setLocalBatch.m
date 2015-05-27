% PROGMANAGER/setLocalBatch - Used to set multiple local variables simultaneously.
%
% SYNTAX
%  setLocalBatch(this, hObject, VARIABLE_NAME, VARIABLE_VALUE, ...)
%   this - @PROGMANAGER
%   hObject - The handle to the program.
%   VARIABLE_NAME - The name of the variable to be set.
%   VARIABLE_VALUE - The value to which to set the variable.
%
% USAGE
%  Specifying multiple name/value pairs at once allows much of the overhead in setting to be handled only once.
%  This optimization is ideal for the end of a function where a number of variables get set.
%
% NOTES
%
% CHANGES
%  TO122205A - Upgrade to Matlab 7.1, case sensitivity. -- Tim O'Connor 12/22/05
%
% Created 7/8/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function setLocalBatch(this, hObject, varargin)
global progmanagerglobal;

% fprintf(1, '--------------setLocalBatch-----------------------------\n%s', getStackTraceString);
% varargin{:}
% fprintf(1, '--------------setLocalBatch-----------------------------\n');

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
    error('Invalid program/gui handle.');
end

if mod(length(varargin), 2) ~= 0
    error('Invalid number of arguments, name value pairs must be provided. Args: %s', num2str(length(varargin)));
end

for i = 1 : 2 : length(varargin) - 1
% if strcmpi(program_name, 'userFcns')
%     if strcmpi(varargin{i}, 'enable')
% getStackTraceString
% varargin{i + 1}
%     end
% end
    progmanagerglobal.programs.(program_name).(gui_name).variables.(varargin{i}) = varargin{i + 1};
    updateGUIsFromVariable(program_name, gui_name, varargin{i});
end

return;