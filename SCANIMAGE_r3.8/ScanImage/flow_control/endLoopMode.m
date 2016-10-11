%% function endLoopMode(flag)
%Handle end/abort of LOOP acquisition mode (end of single LOOP or end of Cycle acquisition, if applicable)
%
%% SYNTAX
%   flag: <Optional; one of {'abort' 'cycleHome'}> If supplied, signals this is an abort or cycle-home end-of-loop
%
%% NOTES
%   Function contains LOOP abort logic previously in executeStartLoopCallback()
%   Logic now used both during user abort as well when numRepeatPeriods value is reached -- see resumeLoop()
%
%% CREDITS
%   Created 11/22/10, by Vijay Iyer
%% ********************************************


function endLoopMode(flag)

global gh state

if nargin < 1 || isempty(flag)
    abort = false;
    cycleHome = false;
else
    abort = strcmpi(flag,'abort');
    cycleHome = strcmpi(flag,'cycleHome');
end

state.internal.looping = 0;
state.cycle.cycling = 0;
state.internal.abortActionFunctions = 1;
state.internal.abort = 1;

hLoopButton = gh.mainControls.startLoopButton;


if abort
    notify(state.hSI,'abortAcquisitionStart'); %VI100410A
end

closeShutter;
setStatusString('Ending Loop...');
set(hLoopButton, 'Enable', 'off');

stopGrab(abort); %Identify whether this is an abort operation
scim_parkLaser;
flushAOData;
setImagesToWhole();

%if ~executeGoHome(true) %VI032010B %VI103009A: Return to cycle home, if it makes sense %VI101008A: Only restore Grab button if no MP285 error caused (or pre-existing)
if abort
    motorAbort(); %DEQ20110112
end

if abort && state.cycle.cycleOn
    motorGoHome('cycleAbort'); %Go to iteration or stack home, as applicable
elseif cycleHome
    motorGoHome('cycleHome'); %Go to cycle home, if any
else
    motorGoHome(); %Just go to stack home, if any
end    

set(hLoopButton, 'Enable', 'on');
%end

if abort
    if isinf(state.acq.numberOfRepeats)
        setStatusString(''); %Don't report aborted loop when in Infinite mode
    else
        setStatusString('Aborted Loop');
    end
else
    setStatusString('Loop Done');
    pause(0.6); %Allow status string to linger a bit    
    setStatusString(''); %Don't report aborted loop when in Infinite mode
end

set([gh.mainControls.focusButton gh.mainControls.grabOneButton], 'Visible', 'On');
turnOnMenus;
set(hLoopButton, 'String', 'LOOP');

%TO12204b - Tim O'Connor: Mark the beams as 'changed', so the data gets
%reput, in the event of using different acquisition
%methods/types/parameters.
state.init.eom.changed(:) = 1;

% restore any cached config file
if state.cycle.cycleOn && state.cycle.restoreOriginalConfig && state.configCache.isKey('CYCLE_CACHE')
	setStatusString('Restoring config...');
	loadCachedConfiguration('CYCLE_CACHE');
	setStatusString('Config restored');
end

if abort
    notify(state.hSI,'abortAcquisitionEnd'); %VI100410A
else
    notify(state.hSI,'loopModeDone');
end
