function [out] = getprogram(prog_object, hobject,variable_name,gui_name,varargin)
% GETPROGRAM   - @progmanager method that gets the value of a variable in specified gui_name in same program as hobject.
%   GETPROGRAM will look at the current progmanagerglobal and get the
%   variable or structure specified in variable_name for the gui specified in gui_name.  
% 	This function uses the location of the object and the gui name to get the correct value
%   from the programmanger.
%
%   varargin are the indices to the array if you want to subsindex into the
%   data stored in the variable.
%
%   Example: ( if gui name was 'main' in program 'scanimage')
%
%         val=getprogram(prog_object,hobject,'name','main');
% 
%   is the same as specifying the program name of the
%   variable using SETGLOBAL.
%                     
%   See also GETGLOBAL, GETMAIN, SETLOCAL, SETGLOBAL, SETMAIN, SETPROGRAM

out=[];
fighandle=getParent(hobject,'figure');

UserData=get(fighandle,'UserData');
program_name=UserData.progname; %  program name.

global progmanagerglobal
if nargin == 3
    out=getglobal(prog_object,variable_name,gui_name,program_name,varargin{:});
else
    error('@progmanager/getlocal: must supply 3 inputs.');
end

