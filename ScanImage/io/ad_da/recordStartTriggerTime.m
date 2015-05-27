function recordStartTriggerTime(sampsSinceStart,roughTime)
%% function recordStartTriggerTime(sampsSinceStart,roughtTime)
% Record start trigger time (or first next trigger, in 'Next Trigger Only' case) to header file, following its 'measurement' and correcting for software event latency error (as best as possible)
%
%% SYNTAX
%   sampsSinceStart: Number of samples since start trigger (or first next trigger, in 'Next Trigger Only' case) occurred, given as a double
%   roughTime: The time, determined by clock(), recorded as close to the 'same' instant that sampsSinceStart is determined
%
%% CREDITS
%   Created 9/2/09, by Vijay Iyer
%
%% ****************************************************************************

global state

%Get corrected trigger time on first sample
state.internal.triggerTime = datevec(addtodate(datenum(roughTime),-round(1000*sampsSinceStart/state.acq.inputRate),'millisecond'));
state.internal.triggerTimeString = clockToString(state.internal.triggerTime);
updateHeaderString('state.internal.triggerTimeString');

%Store trigger time as 'first' trigger time -- (this is useful for gap-free advance-mode next triggered acquistions)
state.internal.triggerTimeFirst = state.internal.triggerTime;
state.internal.triggerTimeFirstString = clockToString(state.internal.triggerTime);
updateHeaderString('state.internal.triggerTimeFirstString');

%On first trigger, triggerFrameDelay is always 0
state.internal.triggerFrameDelayMS = 0;
updateHeaderString('state.internal.triggerFrameDelayMS');