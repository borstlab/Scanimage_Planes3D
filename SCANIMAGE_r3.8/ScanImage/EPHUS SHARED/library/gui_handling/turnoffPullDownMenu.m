function turnoffPullDownMenu(menu, label)
% TURNOFFPULLDOWNMENU   - Disable selected menu by label.
%   TURNOFFPULLDOWNMENU will take the menu handle (class uimenu) and label
%  (string) and turn off the menu item with the name label.
%
% See also TURNONPULLDOWNMENU

% Changes:
% 	TPMOD1 (2/4/04) - Commented.

children=get(menu, 'Children');
for counter=1:length(children)
	if strcmp(get(children(counter), 'Label'), label)
		set(children(counter), 'Enable', 'off');
		return
	end
end
