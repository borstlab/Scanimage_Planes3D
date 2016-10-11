function updatePixelClockGatedEnable(handle)
%% function updatePixelClockGatedEnable

global state

%Override any setting from a CFG file
if ~isempty(state.init.hLineClkCtr) && ~isempty(state.init.hPixelClkCtr) ...
        && strcmpi(state.init.lineClockBoardID,state.init.pixelClockBoardID)
    state.acq.clockExport.pixelClockGatedEnable = 1;
else
    state.acq.clockExport.pixelClockGatedEnable = 0;
end
updateGUIByGlobal('state.acq.clockExport.pixelClockGatedEnable');
