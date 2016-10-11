function updateScanAngleMultiplier(~,~)
%UPDATERSPS_LISTENER Handles changes necessary after an ROI Scan Parameter (RSP) has been changed.

global state

if ~state.hSI.mdlInitialized
    return;
end

updateRSPs();
resetImageProperties();

numChans = length(state.internal.imagehandle);
for i=1:numChans    
    set(state.internal.imagehandle(i),'CData',zeros(state.internal.storedLinesPerFrame,state.acq.pixelsPerLine,'uint16'));    
    set(state.internal.maximagehandle(i),'CData',zeros(state.internal.storedLinesPerFrame,state.acq.pixelsPerLine,'uint16'));
end

set(state.internal.mergeimage,'CData',zeros(state.internal.storedLinesPerFrame,state.acq.pixelsPerLine,3));




