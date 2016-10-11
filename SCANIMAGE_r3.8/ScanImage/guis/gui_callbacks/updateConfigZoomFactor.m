function updateConfigZoomFactor(handle)
%% function updateConfigZoomFactor(handle)
% Callback function that handles update to the zoom factor currently designated for configuration
%
%% NOTES
%   Being an INI-named callback allows this to be called during either a GUI control or CFG/USR file loading event
%
%% CHANGES
%   VI012909A: Handle scan delay differentially between bidi and sawtooth scan modes -- Vijay Iyer 1/29/09
%   VI013009A: Ensure config zoom factor is valid  -- Vijay Iyer 1/30/09
%% CREDITS
%   Created 1/22/09, by Vijay Iyer
%% ******************************************************************

global state gh

%%%VI013009A: Ensure config zoom factor is valid
if state.internal.configZoomFactor > length(state.internal.fillFractionGUIArray)
    state.internal.configZoomFactor = 1;
    updateGUIByGlobal('state.internal.configZoomFactor');
end

%Ensure arrays are as long as required
numNewEntries = state.acq.baseZoomFactor - length(state.internal.fillFractionGUIArray);
if numNewEntries > 0
    for i=1:numNewEntries
        state.internal.fillFractionGUIArray(end+1) = state.internal.fillFractionGUIArray(end);
        state.internal.acqDelayArray(end+1) = state.internal.acqDelayArray(end);
        state.internal.scanDelayArray(end+1) = state.internal.scanDelayArray(end);
    end
end

%Extract vallues for current configuration zoom factor
idx = state.internal.configZoomFactor;
state.internal.fillFractionGUIConfig = state.internal.fillFractionGUIArray(idx);
state.internal.acqDelayConfig = state.internal.acqDelayArray(idx);
if ~state.acq.bidirectionalScan %VI012909A
    state.internal.scanDelayConfig = state.internal.scanDelayArray(idx);
else
    updateBidiScanDelay(); %VI012909A
end 

[junk,state.internal.msPerLineConfig] = decodeFillFractionGUI(state.internal.fillFractionGUIConfig);

%Update GUI values for the specified zoom 'index'
updateGUIByGlobal('state.internal.fillFractionGUIConfig');
updateGUIByGlobal('state.internal.msPerLineConfig');
updateGUIByGlobal('state.internal.acqDelayConfig');
updateGUIByGlobal('state.internal.scanDelayConfig');

%Update array 'strings' for CFG file serialization -- (isn't this done anyway in saveConfigFileFast() ?) 
% state.internal.fillFractionGUIArrayString = mat2str(state.internal.fillFractionGUIArray);
% state.internal.acqDelayArrayString = mat2str(state.internal.acqDelayArray);
% state.internal.scanDelayArrayString = mat2str(state.internal.scanDelayArray);


        
        