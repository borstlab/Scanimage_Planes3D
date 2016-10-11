function iterateLoop()
%% function iterateLoop()
%   Start subsequent Repeat within a LOOP acquisition
%
%% NOTES
%   This function only starts subsequent Repeats -- the first Repeat in a LOOP acquisition is started by initializeLoop()
%
%% CREDITS
%   Created 1/4/11, by Vijay Iyer
%   Based heavily on earlier (now deprecated) function resumeLoop()
%% ************************************************************

global state

% if state.internal.repeatCounter == state.acq.numberOfRepeats
if state.internal.repeatCounter == state.acq.numberOfRepeats - 1 %Just completed final Repeat

    if state.cycle.cycleOn
        iterateCycle();
    else
        %Show that all Repeats have now completed
        state.internal.repeatCounter = state.internal.repeatCounter + 1;
        updateGUIByGlobal('state.internal.repeatCounter');

        endLoopMode();
    end
else
    
    %Return to stack/repeat home, as needed; reset beam power scaling
    motorGoHome(); %VI030811A
    state.init.eom.stackPowerScaling = ones(state.init.eom.numberOfBeams,1); %VI052909A  
        
    %Update repeat counter
    resetCounters(state.internal.repeatCounter+1); %Increments repeat counter
    
     %Update countdown timer
    if state.acq.externallyTriggered
        state.internal.secondsCounter = 0;
        updateGUIByGlobal('state.internal.secondsCounter');
    else
        countdownVal = state.acq.repeatPeriod;
        
        % If about to start last Repeat, and an upcoming 'iteration delay' is defined,
        % have countdown-timer use that instead of the Loop 'repeatPeriod'.
        if state.internal.repeatCounter == state.acq.numberOfRepeats
            nextIteration = state.cycle.iteration + 1;
            if nextIteration > state.cycle.cycleLength && state.cycle.cycleCount < (state.cycle.numCycleRepeats - 1)
                nextIteration = 1;
                iterationDelay = getCycleVar('iterationDelay',nextIteration);
                
                if ~isempty(iterationDelay)
                    countdownVal = iterationDelay;
                end
			elseif ~isempty(getCycleVar('iterationDelay',nextIteration))
				iterationDelay = getCycleVar('iterationDelay',nextIteration);
				if ~isempty(iterationDelay)
                    countdownVal = iterationDelay;
                end
			end
		end
		
        updateCountdownTimer(countdownVal);
    end
  
    % (Previously in resumeLoop) This seems it should be done by iterateCycle(), and shouldn't change in middle of Repeats
    %state.internal.lastRepeatPeriod=state.cycle.cycleTimeDelay(state.internal.positionToExecute); %VI102709A %Stores /last/ posn's delay value, as implied
    
    %% Wait until repeat time is nearly done (previously in resumeLoop() & mainLoop())
    waitForLoopRepeat(state.acq.repeatPeriod);
    
    %     %Allow LOOP 'pause' by FOCUS acq (was in resumeLoop())
    %     set(gh.mainControls.focusButton, 'Visible', 'On');
    %
    %     if ~state.acq.externallyTriggered
    %         waitRepeatPeriod();
    %      end
    %
    %     % No longer allow pausing of LOOP acq
    %     if strcmp(get(gh.mainControls.focusButton, 'String'), 'FOCUS')
    %         set(gh.mainControls.focusButton, 'Visible', 'Off');
    %     else
    %         setStatusString('STOP FOCUS!');
    %         disp('mainLooop:  Interrupting loop because focus was running at trigger time');
    %         state.internal.looping=0;
    %         return
    %     end
        
   
    
    %% Actual start of next Repeat (previously in mainLoop)
    if state.internal.looping && ~state.internal.loopPaused %Loop may have been aborted or paused while waiting
        startLoopRepeat();
    end
end