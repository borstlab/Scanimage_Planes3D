function turnOffExecuteButtons(flagName)
global gh state


set(gh.mainControls.focusButton, 'enable', 'off')
set(gh.mainControls.grabOneButton, 'enable', 'off')

%Allow Cycle to be aborted while configuration is being loaded
if strcmpi(get(gh.mainControls.startLoopButton,'String'),'loop')
    set(gh.mainControls.startLoopButton, 'enable', 'off')
end

set(gh.motorControls.pbGrabOneStack, 'enable', 'off')
drawnow expose; %VI120910A

if nargin>=1    
    if ~ismember(flagName, state.internal.executeButtonFlags)
        state.internal.executeButtonFlags = {state.internal.executeButtonFlags{:} flagName};
    end
end
