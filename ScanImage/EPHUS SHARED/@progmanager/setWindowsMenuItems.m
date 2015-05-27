% progmanager/setWindowsMenuItems - Update the menus that are common across all programs.
%
% SYNTAX
%  setWindowsMenuItems(this)
%  setWindowsMenuItems(this, flag)
%   flag - When set to 'toggle' it prevents recreation of menus, and just makes sure the checkmarks are correct.
%
% USAGE
%
% NOTES
%  This will change such global parameters such as the list of all running programs.
%
% CHANGES
%  TO010808A/TO012508B - Switch to addUiMenuItems.m to free this from a dependency on the signal processing toolbox (why is `addmenu` in that toolbox?!?). -- Tim O'Connor 1/25/08
%
% Created 10/3/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function setWindowsMenuItems(this, varargin)
global progmanagerglobal;

%Just update the checkmarks.
if ~isempty(varargin)
    if strcmpi(varargin{1}, 'toggle')
        for i = 1 : length(progmanagerglobal.internal.shared.windowMenus)
% fprintf('-:: %s ::- \n', upper(get(get(progmanagerglobal.internal.shared.windowMenus(i), 'Parent'), 'Tag')));
            kids = get(progmanagerglobal.internal.shared.windowMenus(i), 'Children');
            for j = 1 : length(kids)
                udata = get(kids(j), 'UserData');
                if ishandle(udata.figureHandle)
                    if strcmpi(get(udata.figureHandle, 'Visible'), 'On')
% fprintf('%s:%s - On\n', get(get(progmanagerglobal.internal.shared.windowMenus(i), 'Parent'), 'Tag'), get(udata.figureHandle, 'Tag'));
                        set(kids(j), 'Checked', 'On');
                    else
% fprintf('%s:%s - Off\n', get(get(progmanagerglobal.internal.shared.windowMenus(i), 'Parent'), 'Tag'), get(udata.figureHandle, 'Tag'));
                        set(kids(j), 'Checked', 'Off');
                    end
                end
            end
        end
        return;
    end
end

%Prune invalid handles.
progmanagerglobal.internal.shared.progmanagerMenus = progmanagerglobal.internal.shared.progmanagerMenus(find(ishandle(progmanagerglobal.internal.shared.progmanagerMenus)));
progmanagerglobal.internal.shared.windowMenus = progmanagerglobal.internal.shared.windowMenus(find(ishandle(progmanagerglobal.internal.shared.windowMenus)));
if isempty(progmanagerglobal.internal.shared.windowMenus)
    %There's nothing to do.
    return;
end

%Clear out anything old.
for i = 1 : length(progmanagerglobal.internal.shared.windowMenus)
    delete(get(progmanagerglobal.internal.shared.windowMenus(i), 'Children'));
end

%Initialize the menu construction variables.
labels = {'prototype&'};
callbacks = {''};
tags = {'prototype'};
separators = {'off'};
accelerators = {''};
figureHandles = [];

programNames = fieldnames(progmanagerglobal.programs);
for i = 1 : length(programNames)
    guiNames = fieldnames(progmanagerglobal.programs.(programNames{i}).guinames);
    for j = 1 : length(guiNames)
        labels{length(labels) + 1} = get(progmanagerglobal.programs.(programNames{i}).guinames.(guiNames{j}).fighandle, 'Name');
        callbacks{length(callbacks) + 1} = {@selectionCallback, progmanagerglobal.programs.(programNames{i}).guinames.(guiNames{j}).fighandle};
        tags{length(tags) + 1} = [get(progmanagerglobal.programs.(programNames{i}).guinames.(guiNames{j}).fighandle, 'Tag') '-CheckedMenuItem'];
        separators{length(separators) + 1} = 'Off';
        accelerators{length(accelerators) + 1} = '';
        figureHandles(length(figureHandles) + 1) = progmanagerglobal.programs.(programNames{i}).guinames.(guiNames{j}).fighandle;
    end
end
% windowMenus = progmanagerglobal.internal.shared.windowMenus
% labels
% callbacks
% tags
% separators
% accelerators
%Create the prototype menu.
% prototype = addmenu(get(progmanagerglobal.internal.shared.windowMenus(1), 'Parent'), 1, labels, callbacks, tags, separators, accelerators);
%TO010808A/TO012508B - Switch to addUiMenuItems.m to free this from a dependency on the signal processing toolbox (why is `addmenu` in that toolbox?!?). -- Tim O'Connor 1/25/08
prototype = addUiMenuItems(get(progmanagerglobal.internal.shared.windowMenus(1), 'Parent'), 1, labels, callbacks, tags, separators, accelerators);
prototype = prototype(1);
m = get(prototype, 'Children');

%Stuff in the handles to the windows inside each associated menu item.
%This is useful when toggling checkmarks.
for i = 1 : length(m)
    userdata.figureHandle = figureHandles(i);
% fprintf(1, 'Setting USERDATA - %s: %s\n', get(m(end - i + 1), 'Tag'), get(userdata.figureHandle, 'Tag'));
    set(m(end - i + 1), 'UserData', userdata);
end

%Move the prototype into submenus.
for i = 1 : length(progmanagerglobal.internal.shared.windowMenus)
    copied = copyobj(m, ones(size(m)) * progmanagerglobal.internal.shared.windowMenus(i));
end

delete(prototype);

%Now go back and set the checkmarks.
setWindowsMenuItems(progmanager, 'toggle');

return;

%------------------------------------------------------------------
function selectionCallback(varargin)

userdata = get(gcbo, 'UserData');
hObject = userdata.figureHandle;

if strcmpi(get(hObject, 'Visible'), 'On')
    set(hObject, 'Visible', 'Off');
else
    set(hObject, 'Visible', 'On');
end
setWindowsMenuItems(progmanager, 'toggle');
% toggleGuiVisibility(progmanager, gcbf, );

return;
% 
% 
%     labels = {'File&', 'Open...', 'Save', 'Save As...', 'Program Manager', 'Exit', 'Exit All'};
% %     cbs = {'', 'disp(''Open...'')', 'disp(''Save'')','disp(''Save As...'')', 'disp(''Program Manager'')', 'disp(''Exit'')', 'disp(''Exit All'')'};
%     cbs = {'', {@genericOpenData, main_fig_handle}, {@genericSaveProgramData, main_fig_handle}, ...
%         {@genericSaveProgramDataAs, main_fig_handle}, {@closeprogram, obj, main_fig_handle}, @closeAllPrograms};
%     tags = {'', 'open', 'close', 'save', 'saveAs', 'progmanagerSubMenu', 'exit', 'exitAll'};
%     sep = {'Off','Off','Off','Off','On','On', 'Off'};
%     accel = {'', 'O', 'S', 'A', '', 'Q', ''};
%     menuHandles = addmenu(main_fig_handle, 1, labels, cbs, tags, sep, accel);