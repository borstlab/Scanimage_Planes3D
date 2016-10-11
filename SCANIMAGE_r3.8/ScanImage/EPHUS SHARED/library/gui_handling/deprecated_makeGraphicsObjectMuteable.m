% makeGraphicsObjectMuteable(hObject, objectUpdatedFcn)
%
% Modify a graphics object (as of now, only a rectangle) to allow dragging/resizing
% using the mouse. This will load variables into the userdata field, which must not
% get corrupted by other code. This function will not change userdata created by other
% code.
%
% hObject - The graphics object to become muteable.
% objectUpdateFcn - A callback, to let outside functions know the object has changed.
%                   Identifying information should be packed into the call to this function
%                   and/or into the graphics handle. This may be empty or a string, a function_handle 
%                   (to a function that takes the handle as it's sole argument), or a cell array.
%
% Created: Tim O'Connor 1/11/05
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function makeGraphicsObjectMuteable(hObject, objectUpdateFcn)

type = get(hObject, 'Type');
if ~strcmpi(type, 'rectangle') & ~strcmpi(type, 'line')
    warning('Currently only implemented for ''Type'': ''rectangle'' and ''line''.');
    return;
end

set(hObject, 'ButtonDownFcn', {@muteableGraphicsObjectButtonDownFcn, objectUpdateFcn});
udata = get(hObject, 'UserData');

if strcmpi(type, 'rectangle')
    udata.makeGraphicsObjectMuteable.lastPosition = get(hObject, 'Position');
else
    xData = get(hObject, 'XData');
    yData = get(hObject, 'YData');
    if ~all(xData == xData(1)) & ~all(yData == yData(1))
        error('Can not drag/stretch a line that is not horizontal or vertical.');
    end
    udata.makeGraphicsObjectMuteable.lastPosition = xData;
    udata.makeGraphicsObjectMuteable.lastPosition(2, 1:end) = yData;
    
    if any(size(udata.makeGraphicsObjectMuteable.lastPosition) > 2)
        warning('When stretching/dragging, all intermediate points on a line will be lost.');
    end
end
set(hObject, 'UserData', udata);

return;

% --------------------------------------------------------------------
function muteableGraphicsObjectButtonDownFcn(hObject, somethingElse, objectUpdateFcn)

udata = get(hObject, 'UserData');
type = get(hObject, 'Type');

f = getParent(hObject, 'figure');
a = getParent(hObject, 'axes');

if strcmpi(type, 'rectangle')
    udata.makeGraphicsObjectMuteable.lastPosition = get(hObject, 'Position');
    currentPosition = get(hObject, 'Position');
else
    xData = get(hObject, 'XData');
    yData = get(hObject, 'YData');
    udata.makeGraphicsObjectMuteable.lastPosition = xData;
    udata.makeGraphicsObjectMuteable.lastPosition(2, 1:end) = yData;
end

aunits = get(a, 'Units');
set(a, 'Units', get(f, 'Units'));

if strcmpi(type, 'line')
    xData = get(hObject, 'XData');
    yData = get(hObject, 'YData');
    if ~all(xData == xData(1)) & ~all(yData == yData(1))
        error('Can not drag/stretch a line that is not horizontal or vertical.');
    end
    mnx = min(xData);
    mny = min(yData);
    currentPosition = [mnx mny max(xData)-mnx max(yData)-mny];
    if currentPosition(3) == 0
        horizontal = 1;
    else
        horizontal = 0;
    end
    currentPosition(find(currentPosition == 0)) = 0.000001 * min(currentPosition(find(currentPosition(3:4) > 0)));
end

if strcmpi(get(f, 'SelectionType'), 'Normal')
    %Drag
    newPosition = figure2axes(a, dragrect(axes2figure(a, currentPosition, 'rectangle')), 'rectangle');
else
    %Stretch
    newPosition = figure2axes(a, rbbox(axes2figure(a, currentPosition, 'rectangle')), 'rectangle');
end

% currentPosition
% newPosition

if strcmpi(type, 'rectangle')
    set(hObject, 'Position', newPosition);
else
    if horizontal
        newPosition(3) = 0;
    else
        newPosition(4) = 0;
    end
    
    if strcmpi(get(f, 'SelectionType'), 'Normal')
        %Drag
        set(hObject, 'XData', [newPosition(1) newPosition(1)+newPosition(3)]);
        set(hObject, 'YData', [newPosition(2) newPosition(2)+newPosition(4)]);
    else
        %Stretch
        if horizontal
            set(hObject, 'YData', [newPosition(2) newPosition(2)+newPosition(4)]);
        else
            set(hObject, 'XData', [newPosition(1) newPosition(1)+newPosition(3)]);
        end
    end
end

set(a, 'Units', aunits);

%Update the userdata.
udata.makeGraphicsObjectMuteable.lastPosition = newPosition;
set(hObject, 'UserData', udata);

if ~isempty(objectUpdateFcn)
    if strcmpi(class(objectUpdateFcn), 'char')
        eval(objectUpdateFcn);
    elseif strcmpi(class(objectUpdateFcn), 'cell')
        feval(objectUpdateFcn{:});
    elseif strcmpi(class(objectUpdateFcn), 'function_handle')
        feval(objectUpdateFcn, hObject);
    end
end

return;