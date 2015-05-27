function updateCountdownTimer(waitPeriod)

global state

countdownTime = waitPeriod - etime(clock,state.internal.stackTriggerTime);
state.internal.secondsCounter=max(round(countdownTime),0.0);

%This strangeness is necessary because setGUIValue() does not use num2str() when setting a 'String' property - and so will sometimes display '-0' for zero values. Decided against changing (fixing) setGUIValue() because it's a very core function -- Vijay Iyer 10/28/09
if state.internal.secondsCounter == 0
    state.internal.secondsCounter = 0;
end

updateGUIByGlobal('state.internal.secondsCounter');

