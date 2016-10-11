function motorError
%MOTORERROR Handler when motor error has occurred

global state

%Abort current acquisition, if one is in progress
abortCurrent(true);

%state.motor.errorCond = 1;
%state.motor.movePending=0; %VI120910A
state.motor.lastPositionRead=[];

% %Reset motor position state/display
% state.motor.absXPosition = [];
% state.motor.absYPosition = [];
% state.motor.absZPosition = [];
% state.motor.absZZPosition = [];
% motorUpdatePositionDisplay(); %this updates position display to empty. relative origin is immaterial

%Signal to GUI that error has occurred
setStatusString('Motor Error!');
turnOffMotorButtons;



