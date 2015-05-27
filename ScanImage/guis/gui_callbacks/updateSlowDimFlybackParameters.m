function updateSlowDimFlybackParameters(handle)
%% function updateSlowDimFlybackParameters(handle)
% Callback function that handles update to the slow dimension flyback parameters
%
%% NOTES
%   Function is an INI-file callback, so it is invoked either upon adjusting the GUI control or loading a CFG file
%
%% CREDITS
%   Created 10/22/09, by Vijay Iyer
%% ******************************************************************
global state gh

if state.acq.slowDimFlybackFinalLine
    set(gh.configurationControls.cbDiscardFlybackLine,'Enable','on');
else
    set(gh.configurationControls.cbDiscardFlybackLine,'Enable','off');
    state.acq.slowDimDiscardFlybackLine = 0;
    updateGUIByGlobal('state.acq.slowDimDiscardFlybackLine');    
end

if state.acq.slowDimDiscardFlybackLine
    state.internal.storedLinesPerFrame = state.acq.linesPerFrame - 1;
else
    state.internal.storedLinesPerFrame = state.acq.linesPerFrame;
end

    




        
        