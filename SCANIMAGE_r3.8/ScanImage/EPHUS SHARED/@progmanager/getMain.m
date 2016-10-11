function [out] = getMain(prog_object, varargin)
%GETMAIN   - @progmanager method that gets the value of a variable tied to the main GUI of a program.
%   GETMAIN will look at the current progmanagerglobal and get the
%   variable or structure specified in variable_name from the main GUI associated with the program
%   specified by prog_name.  This function uses
%   the location of the object and the gui name to get the correct value
%   from the programmanger.
%
%  SYNTAX
%        variable = getMain(OBJ, hObject, variableName)
%        variable = getMain(OBJ, hObject, variableName, [startIndex endIndex])
%        variable = getMain(OBJ, variableName, programName)
%        variable = getMain(OBJ, variableName, programName, [startIndex endIndex])
%
%   varargin are the indices to the array if you want to subsindex into the
%   data stored in the variable.
%
%   Example: ( if the main GUI name 'main' in the program 'scanimage')
%         val=getMain(progmanager,'name',getProgramName(progmanager,gcf));
% 
%         is the same as:
%
%         val=getglobal(progmanager,'name','main','scanimage');
%                     
%   See also GETGLOBAL, GETMAIN, SETLOCAL, SETGLOBAL, SETMAIN
%
%   Changed:
%           Tim O'Connor 6/4/04 - Take a figure handle, since the program_name may not be known. So it'll work like getLocal. TO061404a
%     TO122205A - Upgrade to Matlab 7.1, case sensitivity. -- Tim O'Connor 12/22/05
%     TO060208B - Handle @program objects in addition to figure handles. -- Tim O'Connor 6/2/08
%
%   Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004

out=[];
global progmanagerglobal

if ishandle(varargin{1})
    %This is all new. -- TO061404a
    udata = get(getParent(varargin{1}, 'figure'), 'UserData');
    out = getGlobal(prog_object, varargin{2}, progmanagerglobal.programs.(udata.progname).mainGUIname, udata.progname, varargin{3:end});%TO122205A
elseif strcmpi(class(varargin{1}), 'program')
    %TO060208B
    program_name = get(varargin{1}, 'program_name');
    gui_name = get(varargin{1}, 'main_gui_name');
    out = getGlobal(prog_object, varargin{2}, gui_name, program_name, varargin{3:end});
elseif ischar(varargin{1}) && length(varargin) >= 2
    %Maintain backwards compatibility with the original form of this method. -- TO061404a
    if isprogram(prog_object, varargin{2})
        gui_name=progmanagerglobal.programs.(varargin{2}).mainGUIname;
    else
        %TO060208B
        error('Program name is not a valid program: ''%s''', varargin{2});
    end
    
    if nargin >= 3
        out = getGlobal(prog_object, varargin{1}, gui_name, varargin{2}, varargin{3:end});%TO122205A
    else
        error('@progmanager/getMain: must supply 3 inputs.');
    end
else
    error('Invalid arguments.');
end

return;