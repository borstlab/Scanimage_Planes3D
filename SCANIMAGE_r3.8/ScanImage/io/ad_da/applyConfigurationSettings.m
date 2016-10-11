%% function applyConfigurationSettings
% Applies configuration settings changes, updating state vars and then AI/AO objects accordingly
%% NOTES
% Originally this function was just to apply state changes to the AI/AO objects, and most config state processing was handled elsewhere.
% However, since all configuration changes are funnelled through here, it's the ideal place to put config state processing.
% Because the procecessing is now a bit intensive (see setAcquisitionParameters()), it is no longer viable to invoke that processer for every individual config var cahnge. The changes
%   must be aggregated and processed en masse.
%
% Function does not take into account changes in number of channels acquired
%% CHANGES
%   VI092708A: Add setAcquisitionParameters() call here -- Vijay Iyer 9/27/08
%   VI011509A: (Refactoring) Remove explicit calls to setupAOData()/flushAOData(), as these are now called as part of setupDAQDevices_ConfigSpecific() -- Vijay Iyer 1/15/09
%   VI011609A: Changed state.init.pockelsOn to state.init.eom.pockelsOn -- Vijay Iyer 1/16/09
%   VI012109A: setAcquistionParameters() renamed to updateAcquisitionParameters() (after substantial revision/simplification) -- Vijay Iyer 1/21/09
%   VI012909A: Reset the configurationChanged flag via new function -- Vijay Iyer 1/29/09
%   VI052009A: (REFACTOR) All calls to setupDaqDevices_ConfigSpecifig() also call preallocateMemory() -- Vijay Iyer 5/21/09
%   VI102409A: Flag if acquisition delay is too large for given scan speed/configuration -- Vijay Iyer 10/24/09 
%   VI070110A: Add updateAcquisitionSize() call here -- Vijay Iyer 7/1/10
%   VI102011A: Call setImagesToWhole() after preallocating memory -- avoids error when pixels/line or lines/frame is increased and blanks out the image on all configuration changes -- Vijay Iyer 10/20/11
%
%% ************************************************

function applyConfigurationSettings
global state gh

%setImagesToWhole; %VI102011A: Removed
checkConfigSettings;

stopGrab;
stopFocus;

updateAcquisitionSize(false); %VI070110A: Handles updates to # frames/slices -- defer call to preallocateMemory()

updateAcquisitionParameters; %VI092708A, VI012109A

setupDAQDevices_ConfigSpecific; %NOTE - this calls preallocateMemory() 

validateAcquisitionDelay(); %VI102409A %Show warning dialog, if needed

%preallocateMemory; %VI052109A

%setupAOData; %VI011509A
%flushAOData; %VI011509A

setImagesToWhole(); %VI102011A
resetImageProperties();

resetCounters;
updateHeaderString('state.acq.pixelsPerLine');
updateHeaderString('state.acq.fillFraction');

%state.internal.configurationChanged=0; %VI012909A: This is now handled via resetConfigurationChanged() below

%startPMTOffsets; %VI120511A: Removed

updateShutterDelay;

%TPMODPockels
updatePowerBox;

%This is redundant (done in setupAOData()) -- Vijay Iyer 1/09/09
if state.init.eom.pockelsOn %VI011609A
    state.init.eom.changed(:) = 1;
end

verifyEomConfig;

resetConfigurationChanged(); %VI012909A
