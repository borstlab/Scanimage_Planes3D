function endFocus
%% function endFocus


% Function called at the end of the Focus that will park the laser, close the shutter,
% reset the counters (internal), reset the currentMode, and make the 
% Grab One, Focus, and Loop buttons visible.
%
% Written By: Thomas Pologruto
% Cold Spring Harbor Labs
% January 26, 2001
%
%% MODIFICATIONS
%	TPMOD_1: Modified by Tom Pologruto 1/5/04 - Made to comply with Frame scan selection of Focus Mode.
%   VI011509A: (Refactoring) Remove explicit calls to setupAOData()/flushAOData(), as these are now called as part of setupDAQDevices_ConfigSpecific() -- Vijay Iyer 1/15/09
%   VI020509A: Call turnOnMenusFocus(), rather than turnOnMenus() here at the end of FOCUS -- Vijay Iyer 2/5/09
%   VI090109A: putDataFocus() no longer used with new DAQmx interface -- Vijay Iyer 9/1/09
%   VI090309A: Refresh Mirror AO data after parking beam -- Vijay Iyer 9/3/09
%   VI100410A: Add new built-in EventManager event -- Vijay Iyer 10/4/10
%
%% *****************************************************************************

global state gh


% start TPMOD_3 1/5/04  
if state.internal.forceFocusFrameScan & state.internal.forceFocusFrameScanDone
    state.internal.forceFocusFrameScanDone=0;
    state.hSI.lineScanEnabled=1;
    updateGUIByGlobal('state.acq.linescan');
    state.acq.scanAmplitudeY=0;
    updateGUIByGlobal('state.acq.scanAmplitudeY');
    setupAOData;
    %flushAOData; %VI011509A
end
% end TPMOD_3 1/5/04
setStatusString('Ending Focus...');

closeShutter;

stopFocus;

scim_parkLaser;
flushAOData(); %VI090309A
setImagesToWhole();

%putDataFocus; %VI090109A

resetCounters;

set(gh.mainControls.focusButton, 'Visible', 'On');
set(gh.mainControls.focusButton, 'String', 'FOCUS');
set(gh.mainControls.startLoopButton, 'Visible', 'On');
set(gh.mainControls.grabOneButton, 'Visible', 'On');
turnOnMenusFocus; %VI020509A
setImagesToWhole(); %VI100711A

setStatusString('');

notify(state.hSI,'focusDone'); %VI100410A



