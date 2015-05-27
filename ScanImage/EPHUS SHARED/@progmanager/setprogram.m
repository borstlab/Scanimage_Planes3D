function setprogram(prog_object, hobject, variable_name,gui_name,value,varargin)
% SETPROGRAM   - @progmanager method that sets value of a variable in specified gui_name in same program as hobject.
%   SETPROGRAM will look at the current progmanagerglobal and set the
%   variable or structure specified in variable_name to value for the gui specified in gui_name.  
%   This function uses the location of the object and the gui name to get the correct value
%   from the programmanger, and is useful for callbacks to GUIs.
%
%   varargin are the indices to the array if you want to subsindex into the
%   data stored in the variable.
%
%   Example: (if gui name was 'main' in program 'scanimage')
%
%       setprogram(progmanager, hobject,'name','main','newone');
%        
%   is the same as specifying the program name of the
%   variable using SETGLOBAL.
%            
%   See also SETGLOBAL, GETLOCAL, GETGLOBAL, PROGMANAGER, GETPROGRAM

% Parse the input arguments.

fighandle=getParent(hobject,'figure');
UserData=get(fighandle,'UserData');
program_name=UserData.progname; %  program name.

global progmanagerglobal
if nargin >= 4
    setglobal(prog_object,variable_name,gui_name,program_name,value,varargin{:});
else
    error('setlocal: requires at least 4 inputs (prog_object, hobject, variablename, and value)');
end
