function turnOnExecuteButtons(flagName)

global state gh

if nargin>=1
    [found,flagIdx] = ismember(flagName, state.internal.executeButtonFlags);
    if found
        state.internal.executeButtonFlags(flagIdx) = [];
    end
end

if isempty(state.internal.executeButtonFlags)    
    set(gh.mainControls.focusButton, 'enable', 'on')
    set(gh.mainControls.startLoopButton, 'enable', 'on')
    set(gh.mainControls.grabOneButton, 'enable', 'on')
    if ~isempty(state.motor) && state.motor.motorOn %VI112310A
        set(gh.motorControls.pbGrabOneStack, 'enable', 'on')
    end
end
