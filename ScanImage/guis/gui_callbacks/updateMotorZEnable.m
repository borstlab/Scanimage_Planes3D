function updateMotorZEnable(handle)
%% function updateMotorZEnable(handle)
% Callback function that handles update to the secondary z motor enable checbox
%
%% NOTES
%   Function is an INI-file callback, so it is invoked either upon adjusting the GUI control or loading a CFG file
%
%% CREDITS
%   Created 4/12/11, by Vijay Iyer
%% ******************************************************************
global state gh

%Coerce value, if required
if ~state.motor.motorZOn 
    state.motor.motorZEnable = false; %Must be false if no secondary motor
    updateGUIByGlobal('state.motor.motorZEnable');
elseif ~state.motor.dimensionsXYZZ
    state.motor.motorZEnable = true; %Must be true in XY-Z mode
    updateGUIByGlobal('state.motor.motorZEnable');
end

% ensure the correct step-size textfield is shown
state.hSICtl.toggleUseSecondaryZ();
   
%Ensure correct stack endpoints are displayed
state.motor.stackStart = [];
state.motor.stackStop = [];
updateStackEndpoints(); %VI091912: Update stack endpoints, without updating all of position display, as previously done before VI060512
%motorUpdatePositionDisplay(); %VVV060512