% exportClocks - Configure the various clock signals (frame/line/pixel) to be exported via counter/timers.
%
%% SYNTAX
%   exportClocks()
%   exportClocks(nFrames)
%    nFrames: The number of frames to be acquired. 
%
%% NOTES
%   Refactored from setupDAQDevices_Common.m, with the intention of being called from
%   setupDAQDevices_Common.m, setupDAQDevices_ConfigSpecific.m, and possibly other places.
%
%   Function operates in two modes. 
%       If nFrames is supplied, timing parameters are configured.
%       If nFrames is not supplied, triggering and pulse parameters are configured
%
%   Currently this function is very fast for timing configuration (critical, during startFocus/Grab),
%   but somewhat slow for triggering/pulse configuration (less critical, but would be nice to speed up)
%
%   At moment, clock gating configuration controls have been hidden on Clocks GUI, and:
%       * On boards supporting finite output (X series), 'clock gating' means simply a start-trigger cascade (i.e. frame-line/line-pixel) and is forced ON
%       * On boards not supporting finite output (non X-series), 'clock gating' means a 'pause trigger' cascade and is forced OFF 
%       * Clock gating by external source is not available
%   These restrictions are imposed done because 'pause triggering' feature was not fully understood/vetted 
%   on first iteration for this feature (it seemed to work in some contexts, but not others) -- Vijay Iyer 11/6/10
%
%   As consequence, at this time, X series boards are /required/ to implement:
%       * A guaranteed finite clock generation, i.e. without a possible 'extra' clock tick at end of acquisition
%       * Pixel clock feature. Either gating or finite output is required to implement this feature. Non-X boards support neither. 
%
%   See VI071410C, VI071510A, and TO091210B.
%
%% CHANGES
%   VI110710A: Handle correctly the timing configuration when clock gating is not enabled. This bug was never practically manifesting itself, but fixed here anyway. -- Vijay Iyer 11/7/10
%   VI111110A: BUGFIX - Check boardSupportsFinite before assuming numSamples argument is passed to confgureTiming() -- Vijay Iyer 11/11/10
%   VI111110B: Handle polarity change in clockExportGUI(), not here anymore -- Vijay Iyer 11/11/10
% 
%% CREDITS
%   Tim O'Connor & Vijay Iyer :: 7/14/10 - 9/12/10
%% ************************************************************

function exportClocks(nFrames)
global state
lineTime = 0.001 * state.acq.msPerLine * state.acq.fillFraction;
initialDelay = state.acq.acqDelay + state.acq.scanDelay;

%Frame clock.
try
    if ~isempty(state.init.frameClockBoardID) && state.acq.clockExport.frameClockEnable
        
        hCtr = state.init.hFrameClkCtr;

        if nargin 
            configureTiming(hCtr, 'Frame',nFrames);
        else       
            %Configure clock  triggering
            configureTriggering(hCtr,'Frame');
            
            %Configure pulse params
            offTime = 0.001 * state.acq.msPerLine - lineTime; %off for non-acquiring portion of final line
            onTime = 0.001 * state.acq.msPerLine * state.acq.linesPerFrame - offTime;
            
            if state.acq.slowDimDiscardFlybackLine
                offTime = offTime + 0.001 * state.acq.msPerLine; %off for additional line (off by start of final line)
                onTime = onTime - 0.001 * state.acq.msPerLine; 
            end
            configurePulseParams(hCtr,'Frame', offTime, onTime, initialDelay);
        end
    end
catch
    fprintf(2, 'Failed to configure export of frame clock:\n%s\n', getLastErrorStack);
end

%Line clock.
try
    if ~isempty(state.init.lineClockBoardID) && state.acq.clockExport.lineClockEnable
        
        hCtr = state.init.hLineClkCtr;
        
        if nargin
            
            if state.init.lineClockBoardSupportsFinite                 
                if state.acq.clockExport.lineClockGated && state.acq.slowDimDiscardFlybackLine
                    %We can suppress discarded line in finite/gated case (uses start triggering, not pause triggering)                    
                    nLines = state.acq.linesPerFrame - 1;
                else
                    nLines = nFrames * state.acq.linesPerFrame;
                end
                
                configureTiming(hCtr,'Line', nLines); %VI110710A                
            else
                configureTiming(hCtr,'Line'); %VI110710A
            end


        else
            %Configure triggering
            configureTriggering(hCtr,'Line','Frame');
            
            %Configure pulse params
            onTime = lineTime;
            offTime = 0.001 * state.acq.msPerLine - lineTime;
            configurePulseParams(hCtr,'Line', offTime, onTime, initialDelay);
        end
    end
catch
    fprintf(2, 'Failed to configure export of line clock:\n%s\n', getLastErrorStack);
end

%Pixel clock.
try
    if ~isempty(state.init.pixelClockBoardID) && state.acq.clockExport.pixelClockEnable

        hCtr = state.init.hPixelClkCtr;
        
        if nargin 
            if state.init.pixelClockBoardSupportsFinite
                configureTiming(hCtr,'Pixel', state.acq.pixelsPerLine);
            else %Not clear that it would ever make sense to support this case
                configureTiming(hCtr,'Pixel', round(nFrames * state.acq.linesPerFrame * (1/state.acq.fillFraction) * state.acq.pixelsPerLine));
            end       
        else            
            %Configure clock  triggering
            configureTriggering(hCtr,'Pixel','Line');
            
            %Configure pulse params
            pixelTime = lineTime / state.acq.pixelsPerLine;
            onTime = pixelTime * state.acq.clockExport.pixelClockPulseWidthFraction;
            offTime = pixelTime - onTime;
            
            configurePulseParams(hCtr,'Pixel', offTime, onTime, initialDelay);
        end
    end
catch
    fprintf(2, 'Failed to configure export of pixel clock:\n%s\n', getLastErrorStack);
end

return;


%% HELPER FUNCTIONS

function configureTriggering(hCtr, clockType,autoSourceClockType)
% Handles triggering requirements of any clock Task which is 'gated'
%
%   clockType: One of {'Pixel' 'Frame' 'Line'}
%   autoSourceClockType: <OPTIONAL> One of {'Pixel' 'Frame' 'Line'}
%

global state

if nargin < 3
    autoSourceClockType = '';
end

clockTypeLower = lower(clockType);
autoSourceClockTypeLower = lower(autoSourceClockType);

clockGatedEnable = state.acq.clockExport.([clockTypeLower 'ClockGatedEnable']);
clockGateSource = state.acq.clockExport.([clockTypeLower 'ClockGateSource']);
boardSupportsFinite = state.init.([lower(clockType) 'ClockBoardSupportsFinite']);

%Verify that auto-source is available, if specified
if clockGatedEnable && ~isempty(autoSourceClockType) && state.acq.clockExport.([clockTypeLower 'ClockAutoSource'])
    clockGated = state.acq.clockExport.([autoSourceClockTypeLower 'ClockEnable']);
else
    clockGated = clockGatedEnable;
end
state.acq.clockExport.([clockTypeLower 'ClockGated']) = clockGated;


if clockGated
    
    if boardSupportsFinite
        hCtr.set('pauseTrigType', 'DAQmx_Val_None'); %Disable pause triggering. NOTE: start trigger is always enabled just prior to exportClocks() call.
    else
        hCtr.disableStartTrig(); %Cannot use start/pause triggering at same time (error -200146)
        hCtr.set('pauseTrigType', 'DAQmx_Val_DigLvl');
    end       
    
    if ~isempty(autoSourceClockType) && state.acq.clockExport.([clockTypeLower 'ClockAutoSource'])
        
        %Start/pause trigger serving as gate is tied to polarity of parent clock
        if state.acq.clockExport.([autoSourceClockTypeLower 'ClockPolarityHigh']);
            startTrigEdge = 'DAQmx_Val_Rising';
            pauseTrigWhen = 'DAQmx_Val_Low';
        else
            startTrigEdge = 'DAQmx_Val_Falling';
            pauseTrigWhen = 'DAQmx_Val_High';
        end        
        
        if state.init.([clockTypeLower 'ClockBoardID']) == state.init.([autoSourceClockTypeLower 'ClockBoardID'])
            
            trigSrc = ['ctr' num2str(state.init.([autoSourceClockTypeLower 'ClockCtrID'])) 'InternalOutput'];            
            if boardSupportsFinite
                hCtr.cfgDigEdgeStartTrig(trigSrc,startTrigEdge);
                hCtr.set('startTrigRetriggerable',1);
                %hCtr.channels(1).set('enableInitialDelayOnRetrigger',1);
            else
                hCtr.set('digLvlPauseTrigSrc', trigSrc, 'digLvlPauseTrigWhen', pauseTrigWhen);%We're only active when the gate is high, thus paused when low.
            end
        else
            errordlg(sprintf('Auto-source cannot be set for %s Clock when the %s and %s Clocks are not on the same board', clockType, autoSourceClockType, clockType));
            state.acq.clockExport.([clockTypeLower 'ClockAutoSource']) = 0;
            updateGuiByGlobal(['state.acq.clockExport.' clockTypeLower 'ClockAutoSource'],'Callback',1);
            return;
        end
    else        
        
        %User-supplied gate -- assume positive polarity        
        trigSrc = ['PFI' num2str(clockGateSource)];
        if boardSupportsFinite
            hCtr.cfgDigEdgeStartTrig(trigSrc,'DAQmx_Val_Rising');
            hCtr.set('startTrigRetriggerable',1);
            %hCtr.channels(1).set('enableInitialDelayOnRetrigger',1);
        else            
            hCtr.set('digLvlPauseTrigSrc', trigSrc , 'digLvlPauseTrigWhen', 'DAQmx_Val_Low');%We're only active when the gate is high, thus paused when low.
        end
    end
    
else
    hCtr.set('startTrigRetriggerable',0);
    hCtr.set('pauseTrigType', 'DAQmx_Val_None'); %Disable pause triggering. NOTE: start trigger is always enabled just prior to exportClocks() call.
end

return;

function configureTiming(hCtr, clockType,numSamples)
%   clockType: One of {'Pixel' 'Frame' 'Line'}

global state
    boardSupportsFinite = state.init.([lower(clockType) 'ClockBoardSupportsFinite']);

    if boardSupportsFinite && ~isinf(numSamples) %VI111110A
        hCtr.cfgImplicitTiming('DAQmx_Val_FiniteSamps', numSamples);
    else
        hCtr.cfgImplicitTiming('DAQmx_Val_ContSamps');
    end 

return;

function configurePulseParams(hCtr, clockType, offTime, onTime, initialDelay)
%   clockType: One of {'Pixel' 'Frame' 'Line'}

global state

clockTypeLower = lower(clockType);
phaseAdjust = state.acq.clockExport.([clockTypeLower 'ClockPhaseShiftUS']) * 1e-6;    
clockGated = state.acq.clockExport.([clockTypeLower 'ClockGated']);

if state.acq.clockExport.([clockTypeLower 'ClockPolarityLow'])
    hCtr.channels(1).set('pulseLowTime', onTime);
    hCtr.channels(1).set('pulseHighTime', offTime);
    hCtr.channels(1).set('pulseIdleState', 'DAQmx_Val_High');
else
    hCtr.channels(1).set('pulseLowTime', offTime);
    hCtr.channels(1).set('pulseHighTime', onTime);
    hCtr.channels(1).set('pulseIdleState', 'DAQmx_Val_Low');
end

%%%VI111110B: Removed %%%%
% %Ensure idle state change, if any, takes effect immediately
% hCtr.start();
% hCtr.abort();
%%%%%%%%%%%%%%%%%%%%%%%%%%

%hCtr.channels(1).set('pulseTimeInitialDelay', initialDelay);

if clockGated
    hCtr.channels(1).set('pulseTimeInitialDelay', phaseAdjust); %Don't use delay if clock is gated
else
    hCtr.channels(1).set('pulseTimeInitialDelay', initialDelay + phaseAdjust);
end

return;



