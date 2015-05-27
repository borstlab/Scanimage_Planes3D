%% CHANGES
%   Tim O'Connor 9/26/03 - Allow 'period' delays to apply across loop iterations.
%   VI102008A: Eliminate user of scanLaserBeam variable -- Vijay Iyer 10/17/08
%   VI011609A: Changed state.init.pockelsOn to state.init.eom.pockelsOn -- Vijay Iyer 1/16/09
%   VI012109A: msPerLine is now actually in milliseconds -- Vijay Iyer 1/21/09
%   VI090409A: Use new stopActionFunctions field to prevent excess callbacks -- Vijay Iyer 9/4/09


function executeROICycle(currentAcq,needUpdate)
global state gh

% Tell Software what we are doing....
if needUpdate   %Do we need to change anything this round?
    % Configure power if using a pockels cell....
    if state.init.eom.pockelsOn==1 %VI011609A                
        %if state.init.eom.maxPower(state.init.eom.scanLaserBeam) ~= currentAcq(6) %VI102008A
        %   state.init.eom.changed(state.init.eom.scanLaserBeam)=1; %VI102008A
        
        if state.init.eom.maxPower(state.init.eom.beamMenu) ~= currentAcq(6) %VI102008A
            state.init.eom.changed(state.init.eom.beamMenu)=1; %VI102008A       
            state.init.eom.maxPowerDisplaySlider=currentAcq(6);
            updateGUIByGlobal('state.init.eom.maxPowerDisplaySlider');
            powerControl('maxPower_Slider_Callback',gh.powerControl.maxPower_Slider);
            ensureEomGuiStates;
        end
end
    % Configure normal stuff like frames and averaging....
    state.acq.numberOfFrames=currentAcq(4);
    state.acq.averaging=currentAcq(5);
    state.acq.numberOfZSlices=1;
    state.standardMode.averaging=state.acq.averaging;
    state.standardMode.numberOfZSlices=state.acq.numberOfZSlices;
    state.standardMode.numberOfFrames=state.acq.numberOfFrames;
    updateGUIByGlobal('state.acq.numberOfFrames');
    updateGUIByGlobal('state.acq.averaging');
    updateGUIByGlobal('state.acq.numberOfZSlices');
    updateGUIByGlobal('state.standardMode.numberOfFrames');
    updateGUIByGlobal('state.standardMode.averaging');
    updateGUIByGlobal('state.standardMode.numberOfZSlices');
    alterDAQ_NewNumberOfFrames;
    preallocateMemory;
    % GOTO New ROI....
    roi=currentAcq(3);
    setROIProps(roi); 
else
    stopGrab;
    flushAOData;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Start to wait to keep timing straight....
period=currentAcq(2);
updateGUIByGlobal('state.internal.frameCounter');
state.internal.secondsCounter=period;
updateGUIByGlobal('state.internal.secondsCounter');
%I altered the logic here a bit.
%First, one of the conditions was just whacko (~state.roiCycle.firstTimeThroughLoop == 1)
%Second, the loop wouldn't apply the period across loop iterations. That
%is, it would execute the last ROI in the loop, then jump back to the first
%one, without applying any wait induced by the 'period'. --Tim O'Connor 9/26/03
if state.roiCycle.firstTimeThroughLoop
    state.roiCycle.roiCyclePosition = state.roiCycle.currentPos;
    updateGUIByGlobal('state.roiCycle.roiCyclePosition');
    roiCycleGUI('roiCyclePosition_Callback',gh.roiCycleGUI.roiCyclePosition);
    state.roiCycle.firstTimeThroughLoop = 0;
else
%     %TO022508B - Make this update internally, to move the cycle along. How did this work before? From Physiology, maybe?
%     if ~isfield(state, 'physiology')
%         state.roiCycle.roiCyclePosition = state.roiCycle.roiCyclePosition + 1;
%         if state.roiCycle.roiCyclePosition > size(state.roiCycle.currentROICycle,1)
%             state.roiCycle.roiCyclePosition = 1;
%         end
%         state.roiCycle.currentPos = state.roiCycle.roiCyclePosition;
%         updateGUIByGlobal('state.roiCycle.roiCyclePosition');
%     end
    if state.standardMode.repeatPeriod > .01
        pause(max(0, state.standardMode.repeatPeriod - ... 
            1e-3 * state.acq.msPerLine * state.acq.linesPerFrame * state.acq.numberOfFrames)); %VI012109A
    end
    % this is to insert a delay ...
end

%Tim O'Connor 7/20/04 TO072004b: Make sure all headers are up to date.
updateHeaderForAcquisition;

%Tim O'Connor 7/20/04 TO072004c: Use normal (complicated) power control
%settings for each ROI.
if state.roiCycle.standardPower & state.init.eom.pockelsOn %VI011609A
    for beamCounter = 1 : state.init.eom.numberOfBeams
        putDaqData(state.acq.dm, state.init.eom.pockelsCellNames{beamCounter}, ...
            makePockelsCellDataOutput(beamCounter));
    end
end

if ~isempty(state.roiCycle.lastTimeExecuted)
    while etime(clock,state.roiCycle.lastTimeExecuted) < period
        state.internal.secondsCounter=floor(period-etime(clock,state.roiCycle.lastTimeExecuted));
        updateGUIByGlobal('state.internal.secondsCounter');
        if ~state.internal.roiCycleExecuting
            return
        end
        setStatusString('Waiting ...');
        pause(.001);
    end
end

state.roiCycle.lastTimeExecuted=clock;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state.roiCycle.repeatNumber=state.roiCycle.repeatNumber+1;
updateGUIByGlobal('state.roiCycle.repeatNumber');
state.internal.abortActionFunctions=0;
state.internal.stopActionFunctions=0;
state.internal.stripeCounter=0;
state.internal.forceFirst=1;
resetCounters;
startGrab;
if state.shutter.shutterDelay==0
    openShutter;
else
    state.shutter.shutterOpen=0;
end
setStatusString('Acquiring...');
dioTrigger;
