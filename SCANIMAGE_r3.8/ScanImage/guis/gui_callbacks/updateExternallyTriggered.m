function updateExternallyTriggered(handle)
%% function updateExternallyTriggered
%   Handles updates to externally-triggered state (EXT button on Main Controls)
%
%% NOTES
%   Function is an INI-file callback, so it is invoked either upon adjusting the GUI control or loading a CFG file
%
%   TODO: Add an invisible pushbutton below the EXT trigger
%% CHANGES
%   VI070110A: Disable the EXT triggered button when the numberOfZSlices > 1 -- Vijay Iyer 7/1/10
%   VI090810A: Disable the EXT triggered button when the pure next trigger mode is enabled -- Vijay Iyer 9/8/10
%   VI090810B: Handle case where no external start triggers are available (EXT button is then always disabled) -- Vijay Iyer 9/8/10
%   VI102010A: Turn external trigger control 'off' when pure next trigger is enabled -- what was likely previously intended; update varname to enableExtTriggerButton -- Vijay Iyer 10/20/10
%
%% CREDITS
%   Created 12/31/09, by Vijay Iyer.
%% *******************************************************************************************************

global state gh

if state.acq.pureNextTriggerMode
    enableExtTriggerButton = 'off'; %VI102010A %VI090810A
    state.acq.externallyTriggered = 1;
elseif state.acq.numberOfZSlices > 1
    enableExtTriggerButton = 'off'; %VI070110A
    state.acq.externallyTriggered = 0;
else
    if isempty(state.acq.startTrigInputTerminal)
        enableExtTriggerButton = 'off'; %VI070110A
        state.acq.externallyTriggered = 0;
    else
        enableExtTriggerButton = 'on';
        state.acq.externallyTriggered = state.internal.externallyTriggered;
    end
end

set(gh.mainControls.tbExternalTrig,'Enable',enableExtTriggerButton,'Value',state.acq.externallyTriggered); %VI102010A %VI070110A

%%%%%%%%%%%%%%%

