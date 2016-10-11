function updateRSPs(~,~)
%UPDATERSPS_LISTENER Handles changes necessary after an ROI Scan Parameter (RSP) has been changed.

global state;

if ~state.hSI.mdlInitialized
    return;
end

state.hSI.roiRSP_Listener();

end
