function si_toggleStackAcquisitionAvailability(enable)
%% function si_toggleStackAcquisitonAvailability(enable)
% Enable or disable stack acquisition capability, updating controls and state variables accordingly
%
%% NOTES
%   At moment, only deals with GUI controls on standardModeGUI
%
%   At moment, force # of z slices to 1 when disabling stack acquisition. Ideally would cache the value, rather than force it. -- Vijay Iyer 12/31/09
%
%% CREDITS
%   Created 12/31/09, by Vijay Iyer
%% *********************************************

global state gh

if ~state.motor.motorOn
    enable = 0;
end

stackAcquisitionControls = {'etNumberOfZSlices' 'etZStepPerSlice' 'cbReturnHome'}; %VI123109B

if enable
    controlEnableState = 'on';
else
    controlEnableState = 'off';
    
    %For now, force # of z slices to 1
    state.acq.numberOfZSlices = 1; %VI010611A
    updateGUIByGlobal('state.acq.numberOfZSlices','Callback',1);    
end

state.internal.stackAcquisitionAvailable = enable; %Store an internal flag, though it's not currently used -- Vijay Iyer 12/31/09

for i=1:length(stackAcquisitionControls)
    %set(gh.acquisitionGUI.(stackAcquisitionControls{i}),'Enable',controlEnableState);
    set(gh.motorControls.(stackAcquisitionControls{i}),'Enable',controlEnableState);

end



