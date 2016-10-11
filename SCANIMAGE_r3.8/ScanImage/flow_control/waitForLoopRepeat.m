function waitForLoopRepeat(waitPeriod,warnTooShort)
%% function waitForLoopRepeat(waitPeriod,warnTooShort)
% Function waits till there's 1 second until start of next Repeat and t
%   waitPeriod: Time, in seconds, to wait since start of last Repeat to start upcoming Repeat
%   warnTooShort: <Default=true>

global state gh


if nargin < 2
    warnTooShort = true;
end

%Allow LOOP 'pause' by FOCUS acq
set(gh.mainControls.focusButton, 'Visible', 'On');

if ~state.acq.externallyTriggered
    waitToStart(waitPeriod);
end

% % No longer allow pausing of LOOP acq (if not paused already)
% if ~state.internal.loopPaused
%     if strcmp(get(gh.mainControls.focusButton, 'String'), 'FOCUS')
%         if ~strcmp(get(gh.mainControls.startLoopButton, 'String'), 'LOOP')
%             set(gh.mainControls.focusButton, 'Visible', 'Off');
%         end
%     else
%         setStatusString('Loop Stopped!');
%         fprintf(2,'WARNING: Interrupting LOOP because FOCUS was running at trigger time\n');
%         state.internal.looping=0;
%         return
%     end
% end

    function waitToStart(waitPeriod)
        
        %Function waits till specified interval has elapsed, relative to 'stack trigger time' -- i.e. the acquisition trigger that started the current LOOP Repeat
        %Waits occur with 'pause', to ensure that 1) FOCUS button callback can get through, to allow LOOP 'pausing' and 2) ABORT button callback can get through, to allow LOOP to be terminated
          
        setStatusString('Counting down...');
        if etime(clock,state.internal.stackTriggerTime) > waitPeriod && warnTooShort
            setStatusString('DELAY TOO SHORT!');
            beep;
        end
        
        %Loop to allow for abort operations and to avoid updating countdown timer string more than once per second
        while etime(clock,state.internal.stackTriggerTime) < waitPeriod -1
            if state.internal.loopPaused
                return
            end
            
            %Check if aborted
            if abortDuringWait()
                return;
            end
            
            old=etime(clock,state.internal.stackTriggerTime); %VI102909A
            %			updateMotorPosition;
            while floor(etime(clock,state.internal.stackTriggerTime))<old %VI102909A
                pause(0.01); %
                if state.internal.loopPaused
                    return
                end
                
                %Check if aborted
                if abortDuringWait()
                    return;
                end                                
            end
            
            updateCountdownTimer(waitPeriod); %Updates second counter display
            pause(0.01);
        end
        
        state.internal.secondsCounter=0;
        
    end

end

function tf = abortDuringWait()
global state

tf = state.internal.abort; 
if tf
    state.internal.abort=0;
    state.internal.looping=0;
    resetCounters(0); %resets loop repeat counter
    return
end

end
