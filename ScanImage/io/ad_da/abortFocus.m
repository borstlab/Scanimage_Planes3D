%% function abortFocus
%Handle Focus mode aborts
%% CHANGES
%  VI100608A: Use MP285Clear() instead of MP285Flush() -- Vijay Iyer 10/06/08
%  VI090109A: putDataFocus() no longer used with new DAQmx interface -- Vijay Iyer 9/1/09
%   VI090309A: Refresh Mirror AO data after parking beam -- Vijay Iyer 9/3/09
%   VI090309B: Add code segment to deal with frame scan forced during FOCUS. This code is from (prior) abort clause of executeFocusCallback() to refactor universally here -- Vijay Iyer 9/3/09
%   VI090309C: Identify this as an abort operation -- Vijay Iyer 9/3/09
%   VI032010A: (Changes to use new LinearStageController class) Remove superfluous call to set motor velocity. Remove superfluous call to MP285Clear(). -- Vijay Iyer 3/20/10
%   VI100410A: Add new built-in EventManager event -- Vijay Iyer 10/4/10
%   VI102010A: Remove extra stopFocus() call, which was screwing up focus-during-loop operation -- Vijay Iyer 10/20/10
%
%% ***************************************************************
function abortFocus
global gh state

h=gh.mainControls.focusButton;

state.internal.abortActionFunctions=1;
setStatusString('Aborting Focus...');

closeShutter;
set(h, 'Enable', 'off');
stopFocus(true); %VI090309A
%MP285Clear; %VI032010A: Removed %VI100608A 

scim_parkLaser;
%putDataFocus; %VI090109A
flushAOData(); %VI090309A
setImagesToWhole(); 

set(h, 'String', 'FOCUS');
set(h, 'Enable', 'on');
set(gh.mainControls.startLoopButton, 'Visible', 'On');

%%%VI090309A%%%%%%%%%%%%%%%%
% start TPMOD_3 1/5/04
if state.internal.forceFocusFrameScan
    %TO1904c
    set(gh.mainControls.linescan, 'Enable', 'On');
    
    if state.internal.forceFocusFrameScanDone
        state.internal.forceFocusFrameScanDone=0;
%         state.acq.linescan=1;
		state.hSI.lineScanEnabled = 1;
        updateGUIByGlobal('state.acq.linescan');
        state.acq.scanAmplitudeY = 0;
        updateGUIByGlobal('state.acq.scanAmplitudeY');
        setupAOData;
        %flushAOData; %This is called in setupAOData() now...
    end
end
% end TPMOD_3 1/5/04
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%imageControls('cbShowCrosshair_Callback',gh.imageControls.cbShowCrosshair); %Replaced with updateImageBox() (VI083011A)
%updateImageBox(); %not sure why this was needed here (VI083011A)
setStatusString('');
notify(state.hSI,'focusDone'); %VI100410A

%Prepare for next acquisition, including case of resuming paused LOOP
if ~state.internal.looping
    set(gh.mainControls.grabOneButton, 'Visible', 'On');
    turnOnMenusFocus;    
elseif state.internal.loopPaused %Resume paused LOOP   
    turnOffMenus();
    
    state.internal.abort = 0;
    state.internal.abortActionFunctions = 0;
    state.internal.loopPaused = 0;
    
    cycleTransition = state.cycle.cycling;
    
    if cycleTransition 
        updateCountdownTimer(state.cycle.iterationDelay);
    else
        updateCountdownTimer(state.acq.repeatPeriod);
    end
    
    if state.internal.secondsCounter < 1
        fprintf(2,'WARNING: Interrupting LOOP because FOCUS was running at trigger time\n');
        endLoopMode('abort');
    else
        if cycleTransition
            waitForLoopRepeat(state.cycle.iterationDelay, false); %Don't warn if delay is too short..just start!
            if state.internal.looping && ~state.internal.loopPaused %Might get aborted or paused again
                initializeLoop();
                state.cycle.cycling = 0;
            end
        else
            waitForLoopRepeat(state.acq.repeatPeriod);
            if state.internal.looping && ~state.internal.loopPaused %Might get aborted or paused again
                startLoopRepeat();
            end
        end
            
        %         if state.internal.repeatCounter > 0 %iterateLoop() was 'interrupted'
        %             waitForLoopRepeat(state.acq.repeatPeriod);
        %             if state.internal.looping && ~state.internal.loopPaused %Might get paused again
        %                 startLoopRepeat();
        %             end
        %         elseif state.cycle.cycleCount > 0 %iterateCycle() was 'interrupted'
        %             waitForLoopRepeat(state.cycle.iterationDelay)
        %
        %             if state.internal.looping && ~state.internal.loopPaused %Might get paused again
        %                 initializeLoop();
        %             end
        %         end
    end
    
    %     %MP285Clear; %VI032010: Removed %VI100608A
    %     turnOffMenusFocus;
    %
    %     resetCounters;
    %     state.internal.abortActionFunctions=0;
    %     setStatusString('Resuming cycle...');
    %
    %     %stopFocus(true); %Signal this is an 'abort' operation %VI102010A
    %     updateGUIByGlobal('state.internal.frameCounter');
    %     updateGUIByGlobal('state.internal.zSliceCounter');
    %
    %     state.internal.abort=0;
    %     state.internal.currentMode=3;
    %
    %     mainLoop; %This resumes LOOP acquisition which was 'paused' by this FOCUS acq -- Vijay Iyer 1/4/11
else
    assert(false);
end

