function gui_name = getGUIName(prog_manager_obj,hobject)
%GETGUINAME   -  @progmanager method for outputing the GUI name from figure.
%   GETGUINAME(prog_manager_obj,hobject) returns the name of the
%   GUI that the handle hobject belongs.
%
%   See also SHOWGUIS, SHOWPROGRAMS, GETPROGRAMNAME

fighandle=getParent(hobject,'figure');
UserData=get(fighandle,'UserData');
if isstruct(UserData) & isfield(UserData,'guiname')
    gui_name=UserData.guiname; %  program name.
else
    gui_name='';
end
