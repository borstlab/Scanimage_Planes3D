function uncagingMapperPixelDisplayButtonDownFcn

%Incomplete...
% return;

if ~strcmpi(get(gco, 'Type'), 'line')
    return;
end

currentPoint = get(gca, 'CurrentPoint');

imsize = [state.acq.pixelsPerLine  state.acq.linesPerFrame];
xmag = currentPoint(3) ./ imsize(1);
ymag = currentPoint(4) ./ imsize(2);

currentPoint = currentPoint(1, 1 : 2);

doShift = 0;
if strcmpi(get(gcf, 'SelectionType'), 'normal')
    doShift = 1;
end

if doShift
    finalRect = dragrect([currentPoint(1) (currentPoint(2) - ymag * pos(4)) (xmag * pos(3)) (ymag * pos(4))]);
else    
    finalRect = rbbox([currentPoint(1) (currentPoint(2) - ymag * pos(4)) (xmag * pos(3)) (ymag * pos(4))]);
end

updateGUIByGlobal('state.init.eom.uncagingMapper.x', 'Value', finalRect(1) / state.acq.pixelsPerLine, 'Callback', 1);
updateGUIByGlobal('state.init.eom.uncagingMapper.y', 'Value', finalRect(2) / state.acq.linesPerFrame, 'Callback', 1);
updateGUIByGlobal('state.init.eom.uncagingMapper.duration', 'Value', ...
    (finalRect(1) - xfinalRect(3)) / state.acq.pixelsPerLine * (state.acq.msPerLine * state.acq.fillFraction), 'Callback', 1); %VI012109A

updatePixelDisplay;

return;