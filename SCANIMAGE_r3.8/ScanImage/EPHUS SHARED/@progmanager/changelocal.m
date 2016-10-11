function changelocal(prog_object, hobject, variable_name,fcn_handle,varargin)
% CHANGELOCAL   - @progmanager method that changes value of a variable tied to an object by applying fcn_handle to it.
%   CHANGELOCAL will change the data specified by the variable_name by
%   applying the function passed in fcn_handle to it an dreassigning it.
%
%   varargin are additional inputs to the function passed.
%
%   Example: (if gui name was 'main' in program 'scanimage')
%
%       changelocal(prog_object, hobject,'image',@medfilt2,[3 3]);
%        
%   is the same as specifying:
%
%       current=getlocal(prog_object, hobject,'image');
%       current=medfilt2(image,[3 3]);
%       setlocal(prog_object, hobject,'image',current);
%            
%   See also SETGLOBAL, GETLOCAL, GETGLOBAL, PROGMANAGER

% Parse the input arguments.

fighandle=getParent(hobject,'figure');
UserData=get(fighandle,'UserData');
gui_name=UserData.guiname; % gui name.
program_name=UserData.progname; %  program name.

global progmanagerglobal
if nargin >= 4
    if ischar(fcn_handle)
        fcn_handle=str2func(fcn_handle);
    end
    if isempty(varargin)
        progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)=feval(fcn_handle,...
            progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name));
    else
        progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)=feval(fcn_handle,...
            progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name),(varargin{:}));
    end
else
    error('changelocal: requires at least 4 inputs (prog_object, hobject, variablename, and f_handle)');
end

updateGUIsFromVariable(program_name,gui_name,variable_name);