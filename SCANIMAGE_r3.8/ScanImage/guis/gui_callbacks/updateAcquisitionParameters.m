function updateAcquisitionParameters(handle)
%% function updateAcquisitionParameters
%   Handles updates to several of the acquisition parameters settable in the Configuration gui
%% SYNTAX
%   updateAcquisitionParameters()
%   updateAcquisitionParameters(handle)
%       handle: A GUI handle
%
%% NOTES
%       This function replaces setAcquisitionParameters(), on which it is based (but considerably simplified/revamped).
%
%       Currently this function is linked immediately to msPerLine, pixelsPerLine, linesPerFrame, and AO/AI rate parameters. It is invoked also upon applying /any/ changes in the configuration GUI.
%
%       When this function is invoked as a GUI callback, it operates in 'passive' mode. Real changes only are invoked when called from outside a callback in 'active' mode.
%
%% CHANGES
%   VI040509A: Handle new input/output rate scheme where 1.25MHz and 125kHz are base rates, with several possible multipliers -- Vijay Iyer 4/05/09
%   VI040509A: Hard-code the minimum line period increment to be backwards compatible with previous 1.25MHz/50kHz case  -- Vijay Iyer 4/05/09
%   VI041009A: Handle automatic input/output rate adjustments, tied to ms/line value -- Vijay Iyer 4/10/09
%   VI041209A: Ensure acq delay array comports with constraint imposed by AI rate -- Vijay Iyer 4/12/09
%   VI043009A: updateZoom() now handles frame rate update -- Vijay Iyer 4/30/09
%   VI090109A: Handle changes to use new DAQmx interface -- Vijay Iyer 9/01/09
%   VI102209A: Update slow dimension flyback parameters at end, as linesPerFrame change can impact those -- Vijay Iyer 10/22/09
%   VI102209B: Allow odd linesPerFrame with bidi scanning, but only when discarding final line -- Vijay Iyer 10/22/09
%   VI071310A: Relocated update of DAQ Task sample rates from this function to setupDAQDevices_ConfigSpecific() -- Vijay Iyer 7/13/10
%   VI091710A: Ensure that binFactor is always > 1; this is needed to deal with support for higher pixelsPerLine values -- Vijay Iyer 9/17/10
%   VI091910A: Eliminate use of nested functions, set up in VI091710A. Eliminating them solves Matlab lock-up seen in some cases. Should eventually file an issue report with TMW. -- Vijay Iyer 9/19/10
%   VI112310A: Call to updateMaxLUTValue() from updateBinFactor() -- Vijay Iyer 11/23/10
%
%% CREDITS
%   Created 1/21/09, by Vijay Iyer.
%% *******************************************************************************************************
global state gh


if nargin < 1
    active = true;
else
    if ~ishandle(handle)
        error('Argument must be a valid GUI handle');
    end
    active = false;
end

%%%VI102209B%%%%%%%%%%%%%%%%%%%%%%
if mod(state.acq.linesPerFrame,2) && state.acq.bidirectionalScan
    if ~state.acq.slowDimDiscardFlybackLine
        state.acq.linesPerFrame = state.acq.linesPerFrame -1 ;
        updateGUIByGlobal('state.acq.linesPerFrame');
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI040509A: Decode AO/AI rates%%%%%%
inputRateMultiplier = getInputRateMultiplier(); %VI041009A, VI041209A
outputRateMultiplier = getOutputRateMultiplier(); %VI041009A
state.acq.inputRate = inputRateMultiplier * state.internal.baseInputRate; %VI041209A
state.acq.outputRate = outputRateMultiplier * state.internal.baseOutputRate;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI041209A %%%%%%%%%%%%%%
for i=1:state.acq.baseZoomFactor
    state.internal.acqDelayArray(i) = constrainAcqDelay(state.internal.acqDelayArray(i),true);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%minLinePeriodIncrement = 1/gcd(state.internal.baseInputRate,state.internal.baseOutputRate); %VI021309A %Can increment/decrement line period by 100us and still have integer # of AI/AO samples
%minLinePeriodIncrement = 20e-6; %VI040509A

%Exclude fast scans if not bidirectional scanning
if ~state.acq.bidirectionalScan  && state.internal.msPerLineGUI < state.init.minUnidirectionalLinePeriodGUI
    state.internal.msPerLineGUI = state.init.minUnidirectionalLinePeriodGUI;
    updateGUIByGlobal('state.internal.msPerLineGUI');
end

%%%VI032411A: Removed%%%%%%%%%
% %Determine parameters associated with selected line period
% switch state.internal.msPerLineGUI % 1 = 0.5ms, 2 = 1 ms, 3 = 2ms, 4 = 4 ms, 5 = 8 ms
%     case 1
%         state.acq.samplesAcquiredPerLine = 512 * inputRateMultiplier; %VI040509A
%         state.internal.nominalMsPerLine = 0.5;
%         state.internal.linePeriodIncrement = 1*minLinePeriodIncrement;
%     case 2
%         state.acq.samplesAcquiredPerLine = 1024 * inputRateMultiplier; %VI040509A
%         state.internal.nominalMsPerLine = 1;
%         state.internal.linePeriodIncrement = 2 * minLinePeriodIncrement;
%     case 3
%         state.acq.samplesAcquiredPerLine = 2048 * inputRateMultiplier; %VI040509A
%         state.internal.nominalMsPerLine = 2;
%         state.internal.linePeriodIncrement = 4 * minLinePeriodIncrement;
%     case 4
%         state.acq.samplesAcquiredPerLine = 4096 * inputRateMultiplier; %VI040509A
%         state.internal.nominalMsPerLine = 4;
%         state.internal.linePeriodIncrement = 8 * minLinePeriodIncrement;
%     case 5
%         state.acq.samplesAcquiredPerLine = 8192 * inputRateMultiplier; %VI040509A
%         state.internal.nominalMsPerLine = 8;
%         state.internal.linePeriodIncrement = 16 * minLinePeriodIncrement;
%     otherwise
%         error('The specified line period is greater than currently supported');
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI032411A%%%%
power2Multiplier = 2^(state.internal.msPerLineGUI-1);
if state.init.allowFasterLineScans
    state.internal.nominalMsPerLine = 0.125 * power2Multiplier;
    baseSampAcqPerLine = 128;
else
    state.internal.nominalMsPerLine = 0.5 * power2Multiplier;
    baseSampAcqPerLine = 512;
end
    
state.acq.samplesAcquiredPerLine = baseSampAcqPerLine * power2Multiplier * inputRateMultiplier;
state.internal.linePeriodIncrementFactor =  2 * state.internal.nominalMsPerLine;
%%%%%%%%%%%%%%%%
 
updateGUIByGlobal('state.acq.samplesAcquiredPerLine');
state.internal.activeMsPerLine = 1e3 * state.acq.samplesAcquiredPerLine / state.acq.inputRate;

%Handle changes required when configuration is actually applied and ready-to-use
if active
    %Update lengths of parameter arrays
    state.internal.acqDelayArray(state.acq.baseZoomFactor+1:end) = [];
    state.internal.scanDelayArray(state.acq.baseZoomFactor+1:end) = [];
    state.internal.fillFractionGUIArray(state.acq.baseZoomFactor+1:end) = [];
end

%Computes current FF (line period) and servo delay based on current zoom
updateZoom();

%Update other parameters dependent on these acq parameters
updateConfigZoomFactor(); %This calls updateBidiScanDelay(), if it's needed
updatePixelTime();
%updateFrameRate(); %VI043009A: This now handled via updateZoom()
updateSlowDimFlybackParameters(); %VI102209A
updateBinFactor(); %VI091910A



return;

%% HELPER FUNCTIONS
function updatePixelTime()
global state

state.acq.pixelTime = ((state.acq.fillFraction * 1e-3 * state.acq.msPerLine)/state.acq.pixelsPerLine);
state.acq.pixelTimeGUI = state.acq.pixelTime * 1e6; %Display in us
updateGUIByGlobal('state.acq.pixelTimeGUI');

return;

function updateBinFactor()
global state

%%%VI091710A%%%%%%
computeBinFactor();

if state.acq.binFactor < 1
    while state.acq.binFactor < 1
        state.acq.pixelsPerLineGUI = state.acq.pixelsPerLineGUI - 1;
        state.acq.pixelsPerLine = state.acq.pixelsPerLine / 2;
        computeBinFactor();
    end
    
    updateGUIByGlobal('state.acq.pixelsPerLineGUI','Callback',1); %This will invoke updateAcquisitionParameters() again
    return;
else
    updateGUIByGlobal('state.acq.binFactor');
end
%%%%%%%%%%%%%%%%%

updateMaxLUTValue(); %VI112310A

return;

%%%VI041009A%%%%%%%%%%
function inputRateMultiplier = getInputRateMultiplier()
global state gh

 
maxInputRateMultiplier = state.internal.maxInputRateMultiplier;
if maxInputRateMultiplier == 1 %VI110110A
    inputRateMultiplier = 1;
    state.internal.inputRateGUI = 1;
    updateGUIByGlobal('state.internal.inputRateGUI');
    
    inputRateGUIState = 'inactive';

elseif state.internal.inputRateAutoAdjust
    switch state.internal.msPerLineGUI
        case {1,2,3} %VI112810A
            inputRateMultiplier = 4;
        case 4 %VI112810A
            inputRateMultiplier = 2;
        otherwise
            inputRateMultiplier = 1;
    end
    
    inputRateMultiplier = min(maxInputRateMultiplier,inputRateMultiplier); %VI110110A
    
    updateInputRateGUI(inputRateMultiplier); %VI110110A    
    
    inputRateGUIState = 'inactive';
else
    inputRateMultiplier = 2^(state.internal.inputRateGUI - 1);    
    
    %%%VI110110A%%%%%
    if inputRateMultiplier > maxInputRateMultiplier        
        inputRateMultiplier = min(maxInputRateMultiplier,inputRateMultiplier);
        updateInputRateGUI(inputRateMultiplier);
    end
    %%%%%%%%%%%%%%%%%
    
    inputRateGUIState = 'on';
end

cellfun(@(x)set(eval(x),'Enable',inputRateGUIState),getGuiOfGlobal('state.internal.inputRateGUI'));

return;

%%%VI110110A%%%%%%
function updateInputRateGUI(inputRateMultiplier)

global state

state.internal.inputRateGUI = log2(inputRateMultiplier) + 1;
updateGUIByGlobal('state.internal.inputRateGUI');

return;
%%%%%%%%%%%%%%%%%

function outputRateMultiplier = getOutputRateMultiplier()
global state gh

if state.internal.outputRateAutoAdjust
    switch state.internal.msPerLineGUI
        case {1,2,3} %VI112810A
            outputRateMultiplier = 4;
        case 4 %VI112810A
            outputRateMultiplier = 2;
        otherwise
            outputRateMultiplier = 1;
    end
    state.internal.outputRateGUI = log2(outputRateMultiplier) + 1;
    updateGUIByGlobal('state.internal.outputRateGUI');
    
    outputRateGUIState = 'inactive';
else
    outputRateMultiplier = 2^(state.internal.outputRateGUI - 1);
    outputRateGUIState = 'on';
end

cellfun(@(x)set(eval(x),'Enable',outputRateGUIState),getGuiOfGlobal('state.internal.outputRateGUI'));

return;

function computeBinFactor()
global state
state.acq.binFactor = (state.acq.samplesAcquiredPerLine/state.acq.pixelsPerLine);
return;
%%%%%%%%%%%%%%%%%%%%%%%%








