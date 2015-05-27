function out=getMenuEntry(handle, index)
% GETMENUENTRY   - Returns label in listbox/popupmenu from index.
%   GETMENUENTRY will take the gui handle (class uitool) and an index
%   (integer) and out put the string that corresponds to that index in the
%   GUI. If no index is specified, it will return the currently selected
%   string using the object's current 'Value' property (this is the string
%   you see in the display of a popupmenu).
%
%   This function is very useful when dealing with popupmenus and listboxes
%   in GUIs.
%
% See also FINDMENUINDEX

% Changes:
% 	TPMOD1 (2/4/04) - Rewritten and Commented.

str=get(handle,'String');
if nargin < 2
	index=get(handle,'Value');
end

out=[];
if ~iscellstr(str)
	out=str;
elseif index >= 1 & index <= length(str)
	out=str{index};
end
	
