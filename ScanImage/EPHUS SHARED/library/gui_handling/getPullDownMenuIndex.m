function out=getPullDownMenuIndex(menu, label)
% GETPULLDOWNMENUINDEX   - UIMenu name to index converter.
%   GETPULLDOWNMENUINDEX will take the menu handle (class uimenu) and label
%   (string) and output the index of the label.
%
% See also TURNONPULLDOWNMENU, TURNOFFPULLDOWNMENU, UIMENU

%   Changes:
% 	    TPMOD1 (2/4/04) - Re wrote and Commented.

children=get(menu, 'Children');
allLabels=get(children,'Label');
labelBinaryArray=strcmpi(allLabels,label);
out=find(labelBinaryArray);
if isempty(out)
	out=0;
end
