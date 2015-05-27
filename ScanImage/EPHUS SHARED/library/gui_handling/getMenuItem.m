function h = getMenuItem(f, varargin)
% GETMENUITEM - Given a figure handle, it will return the specified menu item's handle.
%
% SYNTAX
%     h = getMenuItem(f, menuTag, itemTag)
%     h = getMenuItem(f, menuTag, subMenuTag, itemTag) %Locating nested items
%
% ARGUMENTS
%     f - The figure handle.
%     menuTag - The menu on which the item is found (ie. file, settings).
%     itemTag - The tag of the item being searched for.
%
% RETURNS
%     h - The handle to the requested menu item, or [] if it is not found.
%
% CREATED
%     Timothy O'Connor 6/9/04
%     Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
if ~ishandle(f)
    error('Matlab:badopt', 'Must pass in a valid figure handle to `getMenuItem`.');
elseif ~strcmpi(get(f, 'Type'), 'figure')
    error('Matlab:badopt', 'Must pass in a valid figure handle to `getMenuItem`: %s', get(f, 'Type'));
end

h = [];

kids = get(f, 'Children');
for i = 1 : length(varargin)
    index = find(strcmp(get(kids, 'Tag'), varargin{i}));
    if length(varargin) > i
        kids = get(kids(index), 'Children');
    end
end

h = kids(index);

return;