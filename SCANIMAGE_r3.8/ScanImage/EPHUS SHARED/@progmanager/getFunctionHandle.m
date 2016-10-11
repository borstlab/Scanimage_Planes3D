% progmanager/getFunctionHandle - Returns the function_handle for the m-file associated with the main gui.
%
% SYNTAX
%  fObject = getFunctionHandle(progmanager, hObject)
%    hObject - The program handle.
%    fObject - The function handle referencing the m-file associated with the program's main gui.
%
% USAGE
%
% NOTES
%
% CHANGES
%
% Created 9/5/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function fObject = getFunctionHandle(this, hObject)
global progmanagerglobal;

%TO020405b - This makes life a lot easier.
if strcmpi(class(hObject), 'program') %TO122205A
    programName = get(hObject, 'program_name');
    guiName = get(hObject, 'main_gui_name');
    hObject = progmanagerglobal.programs.(programName).(guiName).guihandles.(lower(programName));
elseif ishandle(hObject)
    % Parse the input arguments.
    fighandle=getParent(hObject,'figure');
    UserData=get(fighandle,'UserData');
    guiName=UserData.guiname; % gui name.
    programName=UserData.progname; %  program name.
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

fObject = progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).funchandle;

return;