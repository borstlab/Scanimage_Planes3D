function updateBidirectionalScanning(handle)
%% function updateBidirectionalScanning(handle)
% Callback function that handles update to the bidirectional scanning checkbox
%
%% NOTES
%   Function is an INI-file callback, so it is invoked either upon adjusting the GUI control or loading a CFG file
%
%   Unlike most acquisition parameters, bidi scanning affects controls/values outside of the Configuration GUI
%% CHANGES
%   VI042709A: Disable rotation controls when bidi scanning is activated; force rotation to zero -- Vijay Iyer 4/27/09
%   VI042709B: Ensure that ms/line values are properly constrained when toggling bidirectional scanning off -- Vijay Iyer 4/27/09
%
%% CREDITS
%   Created 1/29/09, by Vijay Iyer
%% ******************************************************************
global state gh

controls = [gh.configurationControls.etScanDelay gh.configurationControls.etScanDelayConfig ...
    gh.configurationControls.pbDecScanDelay gh.configurationControls.pbIncScanDelay ...
    gh.mainControls.scanRotation gh.mainControls.scanRotationSlider gh.mainControls.zeroRotate]; %VI042709A

if state.acq.bidirectionalScan       
    %%%VI042709A%%%%%%%%
    state.acq.scanRotation = 0;
    updateGUIByGlobal('state.acq.scanRotation','Callback',1);
    %%%%%%%%%%%%%%%%%%%%
    for i=1:length(controls)
        set(controls(i),'Enable','off');
    end    
    updateBidiScanDelay(); 

else
    for i=1:length(controls)
        set(controls(i),'Enable','on');
    end
    %%%VI042709B%%%%%%%%
    if state.internal.msPerLineGUI < state.init.minUnidirectionalLinePeriodGUI
        state.internal.msPerLineGUI = state.init.minUnidirectionalLinePeriodGUI;
        updateGUIByGlobal('state.internal.msPerLineGUI','Callback',1);
    %%%%%%%%%%%%%%%%%%%%
    else
        updateConfigZoomFactor();
        updateZoom();
    end

end



        
        