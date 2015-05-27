function turnonPullDownMenu(menu, label)
% TURNONPULLDOWNMENU   - Enable selected menu by label.
%   TURNONPULLDOWNMENU will take the menu handle (class uimenu) and label
%   (string) and turn on the menu item with the name label.
%
% See also TURNOFFPULLDOWNMENU

% Changes:
% 	TPMOD1 (2/4/04) - Commented.

children=get(menu, 'Children');
for counter=1:length(children)
	if strcmp(get(children(counter), 'Label'), label)
		set(children(counter), 'Enable', 'on');
		return
	end
end
