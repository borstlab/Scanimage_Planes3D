function dividelocal(prog_object, hobject, variable_name,value,varargin)
% DIVIDELOCAL   - @progmanager method that changes variable tied to an object by dividing it by value.
%   DIVIDELOCAL will multiply the data specified by the variable_name by the
%   value input.  This allows for matrix multiplication (value is an array
%   with the same size as the target data) as well as multiplication by a
%   scalar.
%
%   varargin are indices to the data stored in the variable and the operation will apply to these
%   indices in the array only.
%
%   Example: (if gui name was 'main' in program 'scanimage')
%
%       dividelocal(prog_object, hobject,'scale',factor);
%        
%   is the same as specifying:
%
%       current=getlocal(prog_object, hobject,'scale');
%       current=current./factor;
%       setlocal(prog_object, hobject,'scale',current);
%            
%   See also ADDLOCAL, TIMESLOCAL, CHANGELOCAL

% Parse the input arguments.

fighandle=getParent(hobject,'figure');
UserData=get(fighandle,'UserData');
gui_name=UserData.guiname; % gui name.
program_name=UserData.progname; %  program name.

global progmanagerglobal
if nargin >= 4
    if isempty(varargin)
        progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)=...
         progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)./value;
    else
        progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)(varargin{:})=...
            progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)(varargin{:})./value;
    end
else
    error('setlocal: requires at least 4 inputs (prog_object, hobject, variablename, and value)');
end

updateGUIsFromVariable(program_name,gui_name,variable_name);