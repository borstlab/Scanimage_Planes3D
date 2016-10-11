function [out] = setMain(prog_object, varargin)
%SETMAIN   - @progmanager method that sets the value of a variable tied to the main GUI of a program.
%   SETMAIN will look at the current progmanagerglobal and set the
%   variable or structure specified in variable_name from the main GUI associated with the program
%   specified by prog_name to the supplied value.  This function uses
%   the location of the object and the gui name to get the correct value
%   from the programmanger.
%
%  SYNTAX
%        getMain(OBJ, hObject, variableName, variable)
%        getMain(OBJ, hObject, variableName, variable, [startIndex endIndex])
%        getMain(OBJ, variableName, programName, variable)
%        getMain(OBJ, variableName, programName, variable, [startIndex endIndex])
%
%   varargin are the indices to the array if you want to subsindex into the
%   data stored in the variable.
%
%   Example: ( if the main GUI name 'main' in the program 'scanimage')
%         val=setMain(progmanager,'name',getProgramName(progmanager,gcf),2);
% 
%         is the same as:
%
%         val=setglobal(progmanager,'name','main','scanimage',2);
%                     
%   See also GETGLOBAL, SETLOCAL, SETGLOBAL
%
%   Changed:
%           Tim O'Connor 6/4/04 - Take a figure handle, since the program_name may not be known. So it'll work like setLocal. TO061404b
%           TO122205A - Upgrade to Matlab 7.1, case sensitivity. -- Tim O'Connor 12/22/05
%     TO060208B - Handle @program objects in addition to figure handles. -- Tim O'Connor 6/2/08
%
%   Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004

global progmanagerglobal
out=[];

if ishandle(varargin{1})
    %This is all new. -- TO061404b
    udata = get(getParent(varargin{1}, 'figure'), 'UserData');
    setGlobal(prog_object, varargin{2}, progmanagerglobal.programs.(udata.progname).mainGUIname, udata.progname, varargin{3}, varargin{4:end});%TO122205A
elseif strcmpi(class(varargin{1}), 'program')
    %TO060208B
    program_name = get(varargin{1}, 'program_name');
    gui_name = get(varargin{1}, 'main_gui_name');
    setGlobal(prog_object, varargin{2}, gui_name, program_name, varargin{3}, varargin{4:end});
elseif ischar(varargin{1}) && length(varargin) >= 3
    %Maintain backwards compatibility with the original form of this method. -- TO061404b
    if isprogram(prog_object, varargin{2})
        gui_name=progmanagerglobal.programs.(varargin{2}).mainGUIname;
    else
        %TO060208B
        error('Program name is not a valid program: ''%s''', varargin{2});
    end
    
    if nargin >= 4    
        setGlobal(prog_object, varargin{1}, gui_name, varargin{2}, varargin{3}, varargin{4:end});%TO122205A
    else
        error('@progmanager/setMain: must supply 4 inputs.');
    end
else
    error('Invalid arguments.');
end

return;