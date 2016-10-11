function varargout=addMenuToFigure(fighandle,menuname,varargin)
% ADDMENUTOFIGURE   - Helper function for creating UIMenus.
%   ADDMENUTOFIGURE adds a uimenu to the figure described by fighandle.
%   It can accept many different callbacks/labels.
%
%   Everytime a label is called from varargin, a new subfield is set.
%   The output is the handle to the menu created.
%
%   Ex: addMenuToFigure(gcf,'New','Label','See Me','Callback','gcf')
%
%   will set a new menu called 'New' in the current figure with one sub
%   menu  called 'See Me' with a callback (string of fhandle).
%
%   See also UIMENU, ADDMENU

if nargin < 2
	error('addMenuToFigure: not enough inputs');
end
if ~ishandle(fighandle)
	error('addMenuToFigure: first input must be a figure handle.');
end
if ~ischar(menuname)
	error('addMenuToFigure: second input must be a string.');
end
f = uimenu('Label',menuname,'Parent',fighandle);	%make the menu
submenusindex=find(strcmpi(varargin,'Label'));	%find number of submenus
for submenuCounter=1:length(submenusindex)
	if submenuCounter < length(submenusindex)
		inputs=[{'Parent',f} varargin(submenusindex(submenuCounter):submenusindex(submenuCounter+1)-1)];
	else
		inputs=[{'Parent',f} varargin(submenusindex(submenuCounter):end)];
	end
	uimenu(inputs{:});
end
if nargout == 1
	varargout{1}=f;
end
