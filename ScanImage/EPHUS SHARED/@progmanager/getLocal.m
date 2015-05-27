function varargout = getLocal(obj, hobject,variable_name,varargin)
% GETLOCAL   - @progmanager method that gets the value of a variable tied to an object.
%   GETLOCAL will look at the current progmanagerglobal and get the
%   variable or structure specified in variable_name.  This function uses
%   the location of the object and the gui name to get the correct value
%   from the programmanger.
%
%   varargin are the indices to the array if you want to subsindex into the
%   data stored in the variable.
%
%   Example: ( if gui name was 'main' in program 'scanimage')
%         val=getlocal(hobject,'name');
% 
%   is the same as specifying the program name of the
%   variable using SETGLOBAL.
%                     
%   See also GETGLOBAL, GETMAIN, SETLOCAL, SETGLOBAL, SETMAIN
%
%  CHANGES
%    Tim O'Connor 2/4/05 TO020405b: Allow program objects to be used as handles.
%  TO032905a: Fixed case sensitivity. -- Tim O'Connor 3/29/05
%  TO091405C: Added some much needed verbosity to an error message. There's a plague of meaningless messages in this object. -- Tim O'Connor 9/15/05
%   TO122205A - Upgrade to Matlab 7.1, case sensitivity. -- Tim O'Connor 12/22/05
%  TO123005K: Issue a nice warning if it looks like the call should have been routed to @progmanager/getLocalBatch. -- Tim O'Connor 12/30/05
%  TO071906B: Give an informative message when the handle is empty or numeric. -- Tim O'Connor 7/19/06
global progmanagerglobal

%TO123005K
if nargout > 1
    fprintf(2, '%s - Warning: Call to ''@progmanager/getLocal'' found too many output arguments. The call was probably intended for ''@progmanager/getLocalBatch''.\n%s\n', ...
        datestr(now), getStackTraceString);
    return;
end

%TO020405b - This makes life a lot easier.
if strcmpi(class(hobject), 'program') %TO122205A
    program_name = get(hobject, 'program_name');
    gui_name = get(hobject, 'main_gui_name');
    hobject = progmanagerglobal.programs.(program_name).(gui_name).guihandles.(lower(program_name));
elseif ishandle(hobject)
    fighandle=getParent(hobject,'figure');
    UserData=get(fighandle,'UserData');
    gui_name=UserData.guiname; % gui name.
    program_name=UserData.progname;
elseif isempty(hobject)
    %TO071906B: Give an informative message when the handle is empty. -- Tim O'Connor 7/19/06
    error('Invalid program/gui handle - handle is empty - [].\n%s', getStackTraceString);
elseif isnumeric(hobject)
    %TO071906B: Give an informative message when the handle is numeric. -- Tim O'Connor 7/19/06
    error('Invalid program/gui handle - handle is numeric but an invalid graphics handle: %s\n%s', num2str(hobject), getStackTraceString);
else
    %TO091405C - This used to just say "Invalid program/gui handle." -- Tim O'Connor
    error('Invalid program/gui handle. Class of expected handle: %s\n%s', class(hobject), getStackTraceString);
end

if nargin >= 3
    varargout{1} = getGlobal(obj,variable_name,gui_name,program_name,varargin{:});%  TO032905a: Fixed case sensitivity. -- Tim O'Connor 3/29/05, %TO123005K
else
    error('@progmanager/getlocal: must supply 3 inputs.');
end

return;