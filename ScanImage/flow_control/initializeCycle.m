function initializeCycle()
%INITIALIZECYCLE Initializes cycle used in cycle mode acquisition

global state gh

if ~state.cycle.cycleOn
    return;
end

% Store current configuration
if state.cycle.restoreOriginalConfig
    cacheConfiguration();
end

% initialize our cycle cache
state.internal.preCycleCache = containers.Map();

if state.cycle.autoReset
	state.cycle.iteration = 1;
	updateGUIByGlobal('state.cycle.iteration');
	state.cycle.cycleCount = 0; %VI030811A: Initialize to 0
	updateGUIByGlobal('state.cycle.cycleCount'); %VI030811A
elseif ~isinf(state.cycle.iterationsPerLoop)
    state.cycle.iterationsPerLoopCounter = 0;
end

%Store cycle home posn and reset iteration home posns
if state.motor.motorOn && state.cycle.iteration == 1 && state.cycle.returnHomeAtCycleEnd
    setStatusString('Defining cycle home'); %VI102809A
    state.internal.cycleInitialMotorPosition = motorGetPosition();
else
    state.internal.cycleInitialMotorPosition = [];
end

state.internal.iterationHomePosn = [];
state.internal.iterationHomePosnLast = [];


iterateCycle(true);

end

