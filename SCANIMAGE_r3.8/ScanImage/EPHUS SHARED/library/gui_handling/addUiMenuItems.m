% addUiMenuItems - Adds items to a gui window's menu.
%
% SYNTAX
%  handles = addUiMenuItems(parent, pos, labels, callbacks, tags, seperators, accelerators)
%   handles - Handles to the created items.
%   parent - The parent figure menu on which to place the item. May be a cell array.
%   pos - The position, in the list of items on a given menu, to place the new items. May be a cell array.
%   callbacks - Callbacks to handle menu item selection. May be a cell array.
%   tags - Tags by which to identify the menu items. May be a cell array.
%   separators -  May be a cell array.
%   accelerators -  May be a cell array.
%
% EXAMPLES
%   labels = {'File&', 'New', 'Open...', 'Save', 'Save As...', 'Program Manager', 'Exit', 'Exit All'};
%   cbs = {'', {@genericNewData, main_fig_handle}, {@genericOpenData, main_fig_handle}, {@genericSaveProgramData, main_fig_handle}, ...
%         {@genericSaveProgramDataAs, main_fig_handle}, '', 'closeprogram(progmanager, gcbf)', @closeAllPrograms};
%   tags = {'fileMenu', 'newMenuItem', 'openMenuItem', 'closeMenuItem', 'saveMenuItem', 'saveAsMenuItem', 'progmanagerSubMenuMenuItem', 'exitMenuItem', 'exitAllMenuItem'};
%   sep = {'Off', 'Off','Off','Off','Off','On','On', 'Off'};
%   accel = {'', 'N', 'O', 'S', 'A', '', 'Q', ''};
%   menuHandles = addUiMenuItems(main_fig_handle, 1, labels, cbs, tags, sep, accel);
%
% NOTES
%  Factored out from @progmanager/startprogram, to be accessible to other code.
%  See TO010808A.
%
% Created - Tim O'Connor 1/8/08
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
function handles = addUiMenuItems(parent, pos, labels, callbacks, tags, separators, accelerators)

%%VI073008A -- this seemed to help for 2008a/b compatibility, but no longer seems necessary
% set(parent,'Visible','off');
% drawnow expose;%TO042309A - Not using expose can cause C-spawned events to fire out of order.
% set(parent,'Visible','on');
%%%%%%%%

if ~strcmpi(class(labels), 'cell')
    handles = uimenu(parent, 'Label', labels, 'Position', pos, 'Callback', callbacks, 'Tag', tags, 'Separator', separators, 'Accelerator', accelerators);
    return;
end

if strcmpi(get(parent, 'Type'), 'figure')
    handles(1) = uimenu(parent, 'Label', labels{1}, 'Position', pos, 'Callback', callbacks{1}, 'Tag', tags{1}, 'Separator', separators{1}, 'Accelerator', accelerators{1});
else
    handles(1) = uimenu(parent, 'Label', labels{1}, 'Position', 1, 'Callback', callbacks{1}, 'Tag', tags{1}, 'Separator', separators{1}, 'Accelerator', accelerators{1});
end
for i = 2 : length(labels)
    handles(i) = uimenu(handles(1), 'Label', labels{i}, 'Position', i - 1, 'Callback', callbacks{i}, 'Tag', tags{i}, 'Separator', separators{i}, 'Accelerator', accelerators{i});
end

return;