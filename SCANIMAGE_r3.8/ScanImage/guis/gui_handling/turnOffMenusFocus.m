function turnOffMenusFocus
% Controls to turn off during FOCUS mode acquisition
%% CHANGES
%   VI101708A: Leave imageGUI controls on during FOCUS acquistion
%   VI011609A: Changed state.init.pockelsOn to state.init.eom.pockelsOn -- Vijay Iyer 1/16/09
%   VI020209A: Disallow config load/save during FOCUS operation
%   VI012511A: Use toggleGUI for cycleGUI controls; no cycle controls on MainControls anymore -- Vijay Iyer 1/25/11
%
%% ***********************************************
global gh state
%Added TP
set(gh.mainControls.Settings,'Enable','off');
set(gh.mainControls.File,'Enable','off');
%set(get(gh.imageGUI.figure1,'Children'), 'Enable', 'Off'); %VI101708A
%toggleGUI(gh.acquisitionGUI.figure1,'Off');
toggleCycleGUI([],[],[],'Off'); %VI012511A
%set(gh.mainControls.cyclePosition, 'Enable', 'Off'); %VI012511A 
%set(gh.mainControls.positionToExecuteSlider, 'Enable', 'Off'); %VI012511A
if state.init.eom.pockelsOn && ~state.init.eom.allowSliderDuringFocus %VI112010A %VI011609A
    set([gh.powerControl.Settings gh.powerControl.maxPower_Slider],'Enable','off');  %TPMODPockels
end

set([gh.configurationControls.pbSaveConfig gh.configurationControls.pbLoadConfig],'Enable','off'); %VI020209A

