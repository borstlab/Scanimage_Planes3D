function setLocalGh(prog_object,hobject,handle_tag,varargin)
% SETLOCALGH   - @progmanager method that sets properties of handle with name TAG handle_tag.
%   SETLOCALGH sets properties of hobject to GUI object with name TAG handle_tag.  This
%   is useful for setting properties of the various GUI objects when they
%   are not tied to a variable (like an axes handle).
%
%   The name of the program and the gui are derived from the handle passed
%   in hObject, making this convenient for local GUI programming.
%            
%   See also GETLOCAL, GETGLOBALGH, SETGLOBALGH
%
%  CHANGES
%    Tim O'Connor 3/28/05 TO032805b: Allow program objects to be used as handles. See TO020405b
%  TO032905a: Fixed case sensitivity. -- Tim O'Connor 3/29/05
%   TO122205A - Upgrade to Matlab 7.1, case sensitivity. -- Tim O'Connor 12/22/05
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

global progmanagerglobal
if nargin >= 3  
    setGlobalGh(prog_object,handle_tag,gui_name,program_name,varargin{:})%  TO032905a: Fixed case sensitivity. -- Tim O'Connor 3/29/05
else
    error('@progmanager/getlocalgh: must supply 3 inputs.  See help for details.');
end
