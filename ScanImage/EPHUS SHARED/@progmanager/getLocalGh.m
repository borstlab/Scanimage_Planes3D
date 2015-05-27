function out=getLocalGh(prog_object,hobject,handle_tag, varargin)
% GETLOCALGH   - @progmanager method that gets handle to GUI object with name TAG handle_tag from parent object hobject.
%   GETLOCALGH gets handle to GUI object with name TAG handle_tag.  This
%   is useful for setting properties of the various GUI objects when they
%   are not tied to a variable (like an axes handle).
%
%   The name of the program and the gui are derived from the handle passed
%   in hObject, making this convenient for local GUI programming.
%
% SYNTAX
%   getLocalGh(progmanager, hObject, handleTag)
%   getLocalGh(progmanager, hObject, handleTag, fieldName)
%     progmanager - The program manager object.
%     hObject - The program's handle.
%     handleTag - The tag of the GUI element to be queried.
%     fieldName - The field to be retrieved.
%
% CHANGES
%   TO020905b: Allow the specification of the property in the function call. -- Timothy O'Connor 2/9/05
%   TO032805b: Allow program objects to be used as handles. See TO020405b -- Tim O'Connor 3/28/05
%   TO122205A - Upgrade to Matlab 7.1, case sensitivity. -- Tim O'Connor 12/22/05
%   TO071906A: Case sensitivity. -- Tim O'Connor 7/19/06
%            
%   See also GETLOCAL, GETGLOBALGH
global progmanagerglobal;

%TO032805b - This makes life a lot easier.
if strcmpi(class(hobject), 'program') %TO122205A
    program_name = get(hobject, 'program_name');
    gui_name = get(hobject, 'main_gui_name');
    hobject = progmanagerglobal.programs.(program_name).(gui_name).guihandles.(lower(program_name));
elseif ishandle(hobject)
    % Parse the input arguments.
    fighandle=getParent(hobject,'figure');
    
    UserData=get(fighandle,'UserData');
    gui_name=UserData.guiname; % gui name.
    program_name=UserData.progname; %  program name.
else
    error('Invalid program/gui handle.');
end

out=[];

if nargin == 3  
    out=getGlobalGh(prog_object,handle_tag,gui_name,program_name);%TO071906A
elseif length(varargin) == 1
    %TO020905b
    handle = getGlobalGh(prog_object, handle_tag, gui_name, program_name);
    out = get(handle, varargin{1});
else
    error('@progmanager/getlocalgh: must supply 3 or 4 inputs.  See help for details.');
end
