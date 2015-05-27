% makeGenericallyResizeable
% 
% makeGenericallyResizeable(f) - Takes a figure handle as an argument.
%
% Turns a typically non-resizeable Matlab GUI into one that obeys
% the windows look and feel when resizing. The object will never get smaller
% than the size it is when it is made resizeable.
%
% Some data will be added to the object's userdata, all of it will be part of
% the dynamicGuiData field.
%
% CHANGES
%  TO050905A: Stop using the 'UserData' field, and pack everything into the callback instead. -- Tim O'Connor 5/9/05
%
% Created: Tim O'Connor 5/28/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
function makeGenericallyResizeable(f)

resizeStruct = [];

resizeStruct.tag = get(f, 'Tag');
resizeStruct.units = get(f, 'Units');
resizeStruct.originalPosition = get(f, 'Position');
resizeStruct.lastPosition = resizeStruct.originalPosition;
resizeStruct.lastKnownGoodPosition = resizeStruct.originalPosition;

childHandles = get(f, 'Children');
for i = 1 : length(childHandles)
    if ~ismember(lower(get(childHandles(i), 'Type')), {'uitoolbar', 'uimenu', 'uicontextmenu'})
        resizeStruct(i + 1).tag = get(childHandles(i), 'Tag');
        resizeStruct(i + 1).units = get(childHandles(i), 'Units');
        resizeStruct(i + 1).originalPosition = get(childHandles(i), 'Position');
        resizeStruct(i + 1).lastPosition = resizeStruct(i).originalPosition;
        resizeStruct(i + 1).lastKnownGoodPosition = resizeStruct(i).originalPosition;
    end
end

set(f, 'Resize', 'On', 'ResizeFcn', {@genericResizeFunction, resizeStruct});

childHandles = get(f, 'Children');

return;

% genericResizeFunction
% 
% genericResizeFunction(hObject) - Takes a figure handle as an argument.
%
% Turns a typically non-resizeable Matlab GUI into one that obeys
% the windows look and feel when resizing. The object will never get smaller
% than the size it is when it is made resizeable.
%
% Some data will be added to the object's userdata, all of it will be part of
% the dynamicGuiData field.
%
% This function is not intended to be called by anything except the object's ResizeFcn callback.
%
% Created: Tim O'Connor 5/28/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
function genericResizeFunction(hObject, eventdata, resizeStruct)

    childHandles = get(hObject, 'Children');

    childHandles = childHandles(find(~ismember(lower(get(childHandles, 'Type')), {'uitoolbar', 'uimenu', 'uicontextmenu'})));
    childUnits = get(childHandles, 'Units');

    figUnits = get(hObject, 'Units');

    set(hObject, 'Units', 'Normalized');
    set(childHandles, 'Units', 'Normalized');

    figorig = resizeStruct(1).originalPosition;
    curr = get(hObject, 'Position');

    %Nope, out of bounds.
    if curr(3) < figorig(3) | curr(4) < figorig(4)
fprintf(1, 'UNDO: curr(3) < figorig(3) = %s, curr(4) < figorig(4) = %s\n', num2str(curr(3) < figorig(3)), num2str(curr(4) < figorig(4)));
        set(hObject, 'Position', resizeStruct(1).lastKnownGoodPosition);
%         undoResize(hObject, resizeStruct);
        movegui(hObject);
        set(childHandles, 'Units', childUnits);
        set(hObject, 'Units', figUnits);
        return;
    end
    
    %Move the gui, regardless of resizing.
    resizeStruct(1).lastKnownGoodPosition(1:2) = curr(1:2);

%Calculate the movement of children.
xchangeProportion = (curr(3)) / (figorig(3));
ychangeProportion = (curr(4)) / (figorig(4));
xchangeAbsolute = (curr(3)) - (figorig(3));
ychangeAbsolute = (curr(4)) - (figorig(4));
xchangeProportion
ychangeProportion
xchangeAbsolute
ychangeAbsolute
%Move 'em.
for i = 1 : length(childHandles)
    type = lower(get(childHandles(i), 'Type'));
    if ismember(type, {'uitoolbar', 'uimenu', 'uicontextmenu'})
        %These have no relevant position.
        continue;
    end
    
    %Watch out for objects missing their user data.
    ud = get(childHandles(i), 'UserData');
    if isempty(ud)
        warning('Gui ''%s'' does not support generic resizing.', get(childHandles(i), 'Tag'));
        continue;
    else
        udata(i) = ud;
    end
    
    orig = udata(i).lastKnownGoodPosition;
    pos = get(childHandles(i), 'Position');
%     udata(i).lastPosition = pos;
%     set(childHandles(i), 'UserData', udata(i));

    if strcmpi(type, 'axes')
        %Move and scale.
        pos(1) = orig(1) * xchangeProportion;%x
        pos(2) = orig(2) * ychangeProportion;%y
        pos(3) = orig(3) * xchangeProportion;
        pos(4) = orig(4) * ychangeProportion;
    elseif strcmpi(type, 'uimenu')
        %Do nothing.
    elseif ismember(lower(get(childHandles(i), 'Style')), {'frame', 'edit', 'listbox'})
        %Move and scale.
        pos(1) = orig(1) * xchangeProportion;%x
        pos(2) = orig(2) * ychangeProportion;%y
        pos(3) = orig(3) * xchangeProportion;
        %Maintain the height of single line edit boxes.
        if ~strcmpi(get(childHandles(i), 'Style'), 'edit') | get(childHandles(i), 'Max') > 1
            pos(4) = orig(4) * ychangeProportion;
        end
    else
        %Move.
        pos(1) = orig(1) * xchangeProportion;%x
        pos(2) = orig(2) * ychangeProportion;%y
    end

%     if pos(1) + pos(3) > curr(1) + curr(3) | pos(2) + pos(4) > curr(2) + curr(4) ...
%             | pos(1) < 1 | pos(2) < 1
% pos
% fprintf(1, 'pos(1) + pos(3) > curr(1) + curr(3) = %s, pos(2) + pos(4) > curr(2) + curr(4) = %s, pos(1) < 1 = %s, pos(2) < 1 = %s\nWhole statement: %s\n\n', ...
%     num2str(pos(1) + pos(3) > curr(1) + curr(3)), ...
%     num2str(pos(2) + pos(4) > curr(2) + curr(4)), num2str(pos(1) < 1), num2str(pos(2) < 1), ...
%     num2str(pos(1) + pos(3) > curr(1) + curr(3) | pos(2) + pos(4) > curr(2) + curr(4) | pos(1) < 1 | pos(2) < 1));
    if pos(1) + pos(3) > 1 | pos(2) + pos(4) > 1
'undo'
pos
curr
        undoResize(hObject, resizeStruct);
        movegui(hObject);
        set(childHandles, 'Units', childUnits);
        set(hObject, 'Units', figUnits);
        return;
    else
        set(childHandles(i), 'Position', pos);
    end

%     if length(r) < i
%         r(i) = rectangle('Position', pos);
%         t(i) = text(pos(1), pos(2), get(childHandles(i), 'Tag'));
%     else
%         if ishandle(r(i))
%             delete(r(i))
%             delete(t(i))
%         end
%         r(i) = rectangle('Position', pos);
%         t(i) = text(pos(1), pos(2), get(childHandles(i), 'Tag'));
%     end
end%for

movegui(hObject);

%This move was good, this is the point to return to in case of future errors.
for i = 1 : length(childHandles)
    resizeStruct(i + 1).lastPosition = get(childHandles(i), 'Position');
%     set(childHandles(i), 'UserData', udata(i));
end
resizeStruct.lastPosition = curr;
set(f, 'ResizeFcn', {@genericResizeFunction, resizeStruct});

set(childHandles, 'Units', childUnits);
set(hObject, 'Units', figUnits);

return;

%------------------------------------------------------------------------------
function undoResize(hObject, resizeStruct)

childHandles = get(hObject, 'Children');

set(hObject, 'Position', resizeStruct(1).lastKnownGoodPosition);
for i = 1 : length(childHandles)
    if ismember(lower(get(childHandles(i), 'Type')), {'uitoolbar', 'uimenu', 'uicontextmenu'})
        %These have no relevant position.
        continue;
    end
    
%     %Watch out for objects missing their user data.
%     udata = get(childHandles(i), 'UserData');
%     if isempty(udata)
%         warning('Gui ''%s'' does not support generic resizing.', get(childHandles(i), 'Tag'));
%         continue;
%     end
    
%     udata = get(childHandles(i), 'UserData');
    set(childHandles(i), 'Position', resizeStruct(i + 1).lastPosition);
end

movegui(hObject);

return;