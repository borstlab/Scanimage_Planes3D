function initializeLoop()
%% function initializeLoop()
% Start a LOOP acquisition (including a LOOP acquisition for a single Cycle iteration, if Cycle mode is ON)

%TODO: Determine what to do, if naything, when state.internal.abort=true

global state

%Reset state vars, including 'internal' repeatPeriod var
%state.internal.repeatCounter = 1;
state.internal.repeatCounter = 0;
state.internal.initialMotorPosition = []; %VI103009E: Initialize this, just in case
updateGUIByGlobal('state.internal.repeatCounter');

%This shouldn't be needed anynmore
% if state.cycle.cycleOn
%     %NOTE: Maybe this shoudl be done elsewhere (i.e. initialize/iterateCycle fcns)
%     state.internal.repeatPeriod=state.cycle.cycleTimeDelay(state.internal.indexToExecute);
% else
%     state.internal.repeatPeriod = state.standardMode.repeatPeriod;
% end

%Update countdown timer
if state.acq.externallyTriggered
    state.internal.secondsCounter = 0;
else
    state.internal.secondsCounter=state.acq.repeatPeriod; 
end
updateGUIByGlobal('state.internal.secondsCounter');

%Update file header 
updateHeaderForAcquisition(); %VI121211A

%Start Repeat!
startLoopRepeat();

end

