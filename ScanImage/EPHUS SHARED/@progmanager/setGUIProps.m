function setGUIProps(prog_object,main_gui_handle,gui_name,varargin)
%SETGUIPROPS   - @progmanager method for setting GUI properties from MainGUI Handle.
%  SETGUIPROPS(prog_object,main_gui_handle,gui_name,varargin) sets properties of the 
%   GUI figure with name gui_name that is in the same program as main_gui_handle.  
%   This is useful for setting properties of the various GUI figures.
%            
%   See also SETGLOBALGH
%
%  Changed:
%
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
global progmanagerglobal
if nargin >= 4   
    UserData=get(main_gui_handle,'UserData');
    program_name=UserData.progname; %  program name.
    if isfield(progmanagerglobal.programs.(program_name).guinames,gui_name) & isfield(progmanagerglobal.programs.(program_name).guinames.(gui_name),'fighandle')
       set(progmanagerglobal.programs.(program_name).guinames.(gui_name).fighandle,varargin{:});
    else
        error(['@progmanager/setGUIProps: invalid GUI name ' gui_name ' in program ' program_name]);
    end
else
    error('@progmanager/setGUIProps: must supply 4 inputs.  See help for details.');
end


