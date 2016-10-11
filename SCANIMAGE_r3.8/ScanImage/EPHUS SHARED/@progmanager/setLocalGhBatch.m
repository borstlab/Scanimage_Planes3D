% PROGMANAGER/setLocalGhBatch - Set a "benign" field across multiple GUI objects, simultaneously.
%
% SYNTAX
%  setLocalGhBatch(progmanager, hObject, tags, propertyName, value, ...)
%    progmanager - The program manager object.
%    hObject - The handle of the gui to be used in setting.
%    tag - The tag of the element to be set. A cell array may be provided to set multiple elements at once.
%    propertyName - The property to be set (may be a cell array).
%                   Multiple pairs of propertyName and propertyValue may be used.
%    propertyValue - The value to which to set the property (if propertyName is a cell array, value must be a cell array of the same length).
%                   Multiple pairs of propertyName and propertyValue may be used.
%
% NOTE
%  This function is intended, specifically, for quickly setting the Enable and Visible properties, which have no effect on
%  the value of any linked variables. Other possible uses include setting the Position, colors, etc. No efficiency will be gained
%  by using this to set properties that are linked to variable values (Value, String, Min, Max). It may, in fact, be less
%  efficient to set such properties with this function. A warning will be issued if an attempt is made.
%
% CHANGES
%  TO090106A - Improved handle interrogation. Copy & paste from @progmanager/setLocal. See TO020405b, TO122205A, TO071906B, and TO091405C. -- Tim O'Connor 9/1/06
%
% Created 11/7/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function setLocalGhBatch(this, hObject, tags, varargin)
global progmanagerglobal;

%TO020405b - This makes life a lot easier.
if strcmpi(class(hObject), 'program') %TO122205A
    programName = get(hObject, 'program_name');
    guiName = get(hObject, 'main_gui_name');
    hObject = progmanagerglobal.programs.(programName).(guiName).guihandles.(lower(programName));
elseif ishandle(hObject)
    % Parse the input arguments.
    fighandle=getParent(hObject,'figure');
    UserData = get(fighandle,'UserData');
    guiName = UserData.guiname; % gui name.
    programName = UserData.progname; %  program name.
elseif isempty(hObject)
    %TO071906B: Give an informative message when the handle is empty. -- Tim O'Connor 7/19/06
    error('Invalid program/gui handle - handle is empty - [].\n%s', getStackTraceString);    
elseif isnumeric(hObject)
    %TO071906B: Give an informative message when the handle is numeric. -- Tim O'Connor 7/19/06
    error('Invalid program/gui handle - handle is numeric but an invalid graphics handle: %s\n%s', num2str(hObject), getStackTraceString);
else
    %TO091405C - This used to just say "Invalid program/gui handle." -- Tim O'Connor
    error('Invalid program/gui handle. Class of expected handle: %s\n%s', class(hObject), getStackTraceString);
end

setGlobalGhBatch(this, programName, guiName, tags, varargin{:});

return;