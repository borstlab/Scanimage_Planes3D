function setupDAQDevices_ConfigSpecific
%% function setupDAQDevices_ConfigSpecific
% Sets the configuration specific properties of AI and AO Tasks
%
%% NOTES
%   Completely rewritten to use new DAQmx interface -- Vijay Iyer 9/6/09
%
%   No longer configure (nor allocate memory for) separate AI Tasks for GRAB and FOCUS operations. Only difference between them is the callback, which can be changed cheaply. -- Vijay Iyer 9/6/09
%
%% CHANGES
% VI011509A: (Refactoring) Moved AO object changes to setupAOData(). Call setupAOData() from this function, as they always go together. -- Vijay Iyer 1/15/09
% VI052109A: Moved selectNumberOfStripes() code into here, as this is only place it's called. -- Vijay Iyer 5/21/09
% VI091009A: Remove change of EraseMode based on number of stripes. Mode of 'none' does provide best performance in all cases, so should not be changed. -- Vijay Iyer 9/10/09
% VI091309A: Unregister everyNSamples callback until after buffer is configured,, to avoid DAQmx Error 200877 -- Vijay Iyer 9/13/09
% VI102209A: Use state.internal.storedLinesPerFrame where appropriate -- Vijay Iyer 10/22/09
% VI102409A: Use state.internal.nominalMsPerLine for determining number of stripes -- Vijay Iyer 10/24/09
% VI071310A: Relocated update of DAQ Task sample rates from updateAcquisitionParameters() to here -- Vijay Iyer 7/13/10
% VI072610A: Revert VI102209A -- striping should be determined by acquired linesPerFrame, not storedLinesPerFrame -- Vijay Iyer 7/26/10
% AL120210A: Protect unregistration/registration of everyN callback during fastRestart to fix segVs with repeated focusing/zooming
% VI022311A: Ensure buffer factor is at least 4, to help avoid DAQmx error of 'attempted to read samples that are no longer available' -- Vijay Iyer 2/23/11
% VI022811A: Relax AL120210A protection, which was preventing fill frac changes in middle of FOCUS acquisitions; we could potentially remove protection entirely; at this time, not observing the segV that was reportedly fixed by AL120210A -- Vijay Iyer 2/26/11
%
%% CREDITS
% Created 9/6/09, by Vijay Iyer
% Based heavily on earlier version by Thomas Pologruto & Bernardo Sabatini, 8/11/03
%% *************************************************************************

global state 

%%%VI071310A: Update sample rates
set(state.init.hAI, 'sampClkRate', state.acq.inputRate);
set(state.init.hAO, 'sampClkRate', state.acq.outputRate);

if state.init.eom.pockelsOn
    set(state.init.eom.hAO, 'sampClkRate', state.acq.outputRate);
end

%Set number of focus frames to ensure proper time regardless of image size...
state.internal.numberOfFocusFrames=ceil(state.internal.focusTime / (state.acq.linesPerFrame * 1e-3 * state.acq.msPerLine)); 

selectNumberOfStripes;	% select number of stripes based on # channels and resolution

% Set up total acquisition duration
actualInputRate = state.init.hAI.sampClkRate; 
state.internal.samplesPerLine = round(actualInputRate * 1e-3 * state.acq.msPerLine); 
state.internal.samplesPerFrame = state.internal.samplesPerLine * state.acq.linesPerFrame;
state.internal.samplesPerStripe = state.internal.samplesPerLine * state.acq.linesPerFrame / state.internal.numberOfStripes;

%Determine everyNSamples and buffering values
everyNSamples = state.internal.samplesPerFrame / state.internal.numberOfStripes;
if ~state.internal.fastRestart || everyNSamples ~= state.init.hAI.everyNSamples %VI022811A % AL120210A %NOTE: Might need to add state.cycle.cycling here as well as another case to exclude -- Vijay Iyer 1/6/11
    state.init.hAI.everyNSamples = []; %VI091309A  
    state.init.hAI.cfgInputBufferVerify(computeBufferNumSamples(actualInputRate,everyNSamples), 2*everyNSamples); 
    state.init.hAI.everyNSamples = everyNSamples; %VI091309A
end
    
% PMT Offset: set up total acquisition duration
% actualInputRate = state.init.hAIPMTOffsets.get('sampClkRate'); 
% totalSamplesInputOffsets = 50*state.acq.samplesAcquiredPerLine;		% acquire 50 lines of Data
% state.init.hAIPMTOffsets.set('sampQuantSampPerChan', totalSamplesInputOffsets); 

%Handle the AO side of things, which includes creating the AO data
setupAOData(); %VI011509A

%%%VI071410A: Setup frame/line/pixel clock outputs
%TODO: Line/Pixel clocks
%TODO: Maybe make polarity/init delay/pulse width user settable
% if ~isempty(state.init.hFrameClkCtr)
% %     lowTime = (1/state.acq.frameRate) - state.internal.clockExportPulseTime;    
% %     initDelay = state.acq.acqDelay + state.acq.scanDelay;
% %     set(state.init.hFrameClkCtr.channels, 'pulseLowTime', lowTime, 'pulseTimeInitialDelay', initDelay);
% end
% if ~isempty(state.init.hFrameClkCtr)
% end
exportClocks();%TO091210B

%%%VI052109A%%%%%%%%%%
function selectNumberOfStripes()
global state 

if any(factor(state.acq.linesPerFrame) ~= 2) || state.acq.disableStriping
    state.internal.numberOfStripes = 1;
else
    targetLinesPerStripe = state.internal.targetUpdatePeriod / (1e-3*state.internal.nominalMsPerLine); %VI102409A

    if targetLinesPerStripe >= state.acq.linesPerFrame
        state.internal.numberOfStripes = 1;
    else
        possibleLinesPerStripe = min(2.^(0:10), state.acq.linesPerFrame);
        idx = find(targetLinesPerStripe <= possibleLinesPerStripe,1);

        state.internal.numberOfStripes = state.acq.linesPerFrame / possibleLinesPerStripe(idx);
    end

end

%%%VI091090A: Removed %%%%%%%%
% %If not striping, might as well use 'normal' erase mode, which actually benchmarks as faster
% imageHandles = [state.internal.imagehandle state.internal.mergeimage];
% if state.internal.numberOfStripes == 1
%     set(imageHandles,'EraseMode','normal');
% else
%     set(imageHandles,'EraseMode','none');
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

preallocateMemory(); %VI052109B

return;
%%%%%%%%%%%%%%%%%%%%%

%Avoid DAQmx error -200877 by ensuring that buffer size is an even multiple of the everyNSamples value
function bufferNumSamples = computeBufferNumSamples(inputRate, everyNSamples)
global state

bufferFactor = ceil((inputRate * state.internal.pmtInputBufferTime) / everyNSamples);
if mod(bufferFactor,2) 
    bufferFactor = bufferFactor + 1;
end
bufferFactor = max(bufferFactor,4); %VI022311A: Ensure buffer factor is at least 4

bufferNumSamples = bufferFactor * everyNSamples;

return;
%%%%%%%%%%%%%%%%%%%%%%%%%
    
