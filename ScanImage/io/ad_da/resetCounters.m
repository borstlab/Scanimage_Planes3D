function resetCounters(newRepeatCount)
% Function that resets the counters for a new acquisition at the end of acquisitions or ABORT.

global state gh

state.internal.focusFrameCounter = 0;
state.internal.frameCounter = 0;
state.internal.zSliceCounter = 0;

updateGUIByGlobal('state.internal.frameCounter');
updateGUIByGlobal('state.internal.zSliceCounter');

%Update repeatCounter, if specified
%newRepeatCount can either be the actual new value, or a handle which signifies to reset the repeat count
if nargin
    if round(newRepeatCount) == newRepeatCount %integer value
        state.internal.repeatCounter = newRepeatCount;
        updateGUIByGlobal('state.internal.repeatCounter');
    elseif ishandle(newRepeatCount)
        %state.internal.repeatCounter = 1;
		state.internal.repeatCounter = 0;
        updateGUIByGlobal('state.internal.repeatCounter');
    else
        assert(false);
    end
end
    
%Internal counters, etc:
state.internal.storedFrameCounter = 0;
state.internal.totalFrameCounter = 0; %VI092209A 
state.internal.stripeCounter=0;
state.internal.stripeCounter2=0;
state.internal.inputChannelCounter = 1;




