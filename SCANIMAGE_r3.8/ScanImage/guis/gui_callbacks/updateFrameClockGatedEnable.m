function updateFrameClockGatedEnable(handle)
%% function updateFrameClockGatedEnable

global state

%Override any setting loaded from a CFG file
state.acq.clockExport.frameClockGatedEnable = 0;
updateGUIByGlobal('state.acq.clockExport.frameClockGatedEnable');
