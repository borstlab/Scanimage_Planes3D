function h=getMenuHandles(program_manager,program_id,varargin)
% GETMENUHANDLES  - Returns handles to graphical menu objects from specified program.
% 	GETMENUHANDLES(program_manager,program_id) returns handles to menus in 
% 	main gui of program specified by program_id.
%
% 	GETMENUHANDLES(program_manager,program_id,'Param','Value') turns handles to menus in 
% 	main gui of program specified by program_id with other specified params
% 	define din varargin.
% 	
% 	See also

h=[];
[prog_name]=parseProgramID(program_id);
if isempty(prog_name)
    return
else
    main_gui_name=getProgramProp(program_manager,prog_name,'mainGUIname');
    fig_handle=getHandleFromName(program_manager,main_gui_name,prog_name);
    allchildren=allchild(fig_handle);
    uitoolbar_handles=findobj(allchildren,'type','uitoolbar');
    uipushtool_handles=findobj(allchild(uitoolbar_handles),'type','uipushtool');
    uimenu_handles=findobj(allchildren,'type','uimenu');
    if nargin == 2
        h=findobj([uimenu_handles' uipushtool_handles']);
    else
        h=findobj([uimenu_handles' uipushtool_handles'],varargin{:});
    end
end