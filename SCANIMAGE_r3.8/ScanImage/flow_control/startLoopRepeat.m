function startLoopRepeat()
%STARTLOOPREPEAT Start next Repeat within a Loop acquisition
%
%% NOTES
%   This function draws on earlier logic previously within (deprecated) mainLoop() function -- Vijay Iyer 1/4/11
%
%% CREDITS
%   Created 1/4/11, by Vijay Iyer
%% ***********************************************

global state gh

%Prevent further Focus operation
set(gh.mainControls.focusButton,'Visible','off');

%Store initial position of stack
if state.acq.numberOfZSlices > 1 && state.motor.motorOn	% && state.acq.returnHome
    state.internal.initialMotorPosition = motorGetPosition();
else
    state.internal.initialMotorPosition=[];    
end


%%%VI092909D: Relocated%%%%%%
%     if (state.internal.firstTimeThroughLoop || state.internal.lastRepeatPeriod<=0 ) && ~state.standardMode.standardModeOn %VI102709A %VI110308B
%         state.internal.lastRepeatPeriod = 0;
%         %state.internal.lastTimeDelay=state.cycle.cycleTimeDelay(state.internal.positionToExecute); %VI102709B
%     end
%%%%%%%%%%%%%%%%%%%%%%%%


%Allows LOOP abort to still slip in (needed or not??)
%state.internal.stopAcq = 0; Updated to match berbnardo's code
if state.internal.abort	% Updaeted via BS on 1/16/02
    state.internal.abort=0;
    %state.internal.firstTimeThroughLoop=1;
    resetCounters(); 
    out=0;
    set(gh.mainControls.focusButton, 'Visible','On');
    return
end

%% The Real stuff

%Why is this section here???
if state.init.eom.pockelsOn %VI011609A
    for i = 1 : state.init.eom.numberOfBeams
        if length(state.init.eom.showBoxArray < i)
            continue;
        end
        if state.init.eom.showBoxArray(i)
            state.init.eom.powerBoxWidthsInMs(i) = round(100 * state.init.eom.powerBoxNormCoords(i, 3) ... %VI012109A
                * state.acq.msPerLine / state.acq.pixelsPerLine) / 100;
        else
            state.init.eom.powerBoxWidthsInMs(i) = 0;
        end
    end
    if length(state.init.eom.showBoxArray) < state.init.eom.numberOfBeams
        state.init.eom.showBoxArray(state.init.eom.numberOfBeams) = 0;
    end

    updateHeaderString('state.init.eom.showBoxArray');
    updateHeaderString('state.init.eom.powerBoxWidthsInMs');

end

%setStatusString('Acquiring...'); %VI041308A

state.internal.forceFirst=1;
resetCounters;

% %%VI101308A%%%%
if state.motor.motorOn && (state.acq.numberOfZSlices > 1 || state.cycle.cycleOn) 
    %if MP285RobustAction(@()MP285SetVelocity(state.motor.velocitySlow,1), 'set motor velocity at start of stack', mfilename) %VI101508A
    hMotor = state.motor.hMotor;
    %motorAction(@()set(hMotor,'twoStepMoveEnable',false));
    
    if state.acq.numberOfZSlices > 1    
        % if 'stack position is start center', calculate the z-offset % TODO: this is duplicated logic from executeGrabOneCallback...refactor?
        if state.acq.stackCentered
            state.acq.stackCenteredOffset = floor(state.acq.numberOfZSlices/2.0);
            
            if state.motor.motorZEnable && state.motor.dimensionsXYZZ
                motorSetPositionAbsolute(state.internal.initialMotorPosition - [0 0 0 state.acq.stackCenteredOffset],'verify');
            else
                motorSetPositionAbsolute(state.internal.initialMotorPosition(1:3) - [0 0 state.acq.stackCenteredOffset],'verify');
            end
            
        else
            state.acq.stackCenteredOffset = 0;
        end
    end
end

startGrab;

%%%VI120208A%%%%
if state.internal.abort
    return;
end
%%%%%%%%%%%%%%%%

if state.shutter.shutterDelay==0
    openShutter(~state.acq.externallyTriggered);
else
    state.shutter.shutterOpen=0;
end

% Wait for repeatPeriod to elapse before sending trigger
% NOTE: This is not placed into iterateLoop() -- it's most efficient to do it last, just before sending trigger
if state.internal.repeatCounter > 0
    waitToStart(state.acq.repeatPeriod)
elseif state.cycle.cycleCount > 0
    waitToStart(state.cycle.iterationDelay)
end

dioTriggerConditional; %VI021908A

    function waitToStart(waitPeriod)
        if ~state.acq.externallyTriggered
            if ~isempty(state.internal.stackTriggerTime)
                %Wait for  countdown to reach 0 (should be <1s at this point), leaving small amount of time for code leading up to actual trigger
                countdownTime = waitPeriod-etime(clock,state.internal.stackTriggerTime); %VI102709A
                pause(countdownTime-state.internal.timingDelay);
            end
            
            state.internal.secondsCounter=0; %Ensure that 0 is displayed at end of countdown
            updateGUIByGlobal('state.internal.secondsCounter');
        end
        
    end


end

