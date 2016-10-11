function updateScanAngleMultiplierSlow(~,~)
%UPDATERSPS_LISTENER Handles changes necessary after an ROI Scan Parameter (RSP) has been changed.

global state gh;

if ~state.hSI.mdlInitialized
    return;
end

updateScanAngleMultiplier();

%Handle Toggle Linescan control
baseSAMSlow = state.hSI.roiDataStructure(state.hSI.ROI_BASE_ID).RSPs.scanAngleMultiplierSlow;

if state.acq.scanAngleMultiplierSlow == 0
    set(gh.mainControls.tbToggleLinescan,'Enable','on','Value',1);    
elseif state.acq.scanAngleMultiplierSlow == baseSAMSlow    
    set(gh.mainControls.tbToggleLinescan,'Enable','on','Value',0);    
else
    set(gh.mainControls.tbToggleLinescan,'Enable','off');
end





