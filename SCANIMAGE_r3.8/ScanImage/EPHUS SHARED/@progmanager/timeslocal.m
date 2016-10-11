function timeslocal(prog_object, hobject, variable_name,value,varargin)
% TIMESLOCAL   - @progmanager method that changes variable tied to an object by multiplying it by value.
%   TIMESLOCAL will multiply the data specified by the variable_name by the
%   value input.  This allows for matrix multiplication (value is an array
%   with the same size as the target data) as well as multiplication by a
%   scalar.
%
%   varargin are indices to the data stored in the variable and the operation will apply to these
%   indices in the array only.
%
%   Example: (if gui name was 'main' in program 'scanimage')
%
%       timeslocal(prog_object, hobject,'scale',factor);
%        
%   is the same as specifying:
%
%       current=getlocal(prog_object, hobject,'scale');
%       current=factor.*current;
%       setlocal(prog_object, hobject,'scale',current);
%            
%   See also ADDLOCAL, DIVIDELOCAL, CHANGELOCAL

% Parse the input arguments.

fighandle=getParent(hobject,'figure');
UserData=get(fighandle,'UserData');
gui_name=UserData.guiname; % gui name.
program_name=UserData.progname; %  program name.

global progmanagerglobal
if nargin >= 4
    if isempty(varargin)
        progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)=value.*...
            progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name);
    else
        progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)(varargin{:})=value.*...
            progmanagerglobal.programs.(program_name).(gui_name).variables.(variable_name)(varargin{:});
    end
else
    error('setlocal: requires at least 4 inputs (prog_object, hobject, variablename, and value)');
end

updateGUIsFromVariable(program_name,gui_name,variable_name);