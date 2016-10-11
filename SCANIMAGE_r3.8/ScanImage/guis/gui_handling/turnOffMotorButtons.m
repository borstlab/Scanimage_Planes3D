%% function turnOffMotorButtons
%   Turns off motor control buttons, e.g during an action or following an error
%% CHANGES 
% VI100708A: Handle error condition use case -- Vijay Iyer 10/07/08
% VI100808A: Handle button disabling programatically. Add Grab and stack start/stop buttons to list. -- Vijay Iyer 10/08/08
% VI103008A: Add all the motorControls controls to the list -- Vijay Iyer 10/30/08
% VI032010A: (Changes to use new LinearStageController class) Use motor object's error condition directly -- Vijay Iyer 3/20/10
% VI040210A: Add checkbox motorControls controls to list of  'buttons' -- Vijay Iyer 4/2/10
% VI040210B: Add all text labels and stack endpoint edit text controls to list of controls to disable -- Vijay Iyer 4/2/10
%
%% ************************************************

function turnOffMotorButtons
global state gh

hMotor = state.motor.hMotor; %VI032010A

%%%VI100808A, VI103008A
buttons = {'etPosX' 'etPosY' 'etPosZ' 'pbZeroZ' 'pbZeroXY' 'pbZeroXYZ' ...
    'pbReadPos' 'pbGrabOneStack' 'pbSetStart' 'pbSetEnd'...
    'cbLockSliceVals' 'cbOverrideLz' 'cbReturnHome' 'cbCenteredStack' ...
    'etStepSizeX' 'etStepSizeY' 'etStepSizeZ' ...
    'pmPosnID' 'pbAddCurrent' 'tbTogglePosn' ...
    'pbStepXInc' 'pbStepXDec' 'pbStepYInc' 'pbStepYDec' 'pbStepZInc' 'pbStepZDec' ...
    'cbSecZ' 'pbAltZeroXY' 'pbAltZeroZ' 'etPosZZ'}; %VI051211A %VI040210A;

textLabels = {};
%textLabels = {'stStackEndpointsDominate' 'stOverrideLz'};  %VI04010B
stackEndpoints = {'etStackStart' 'etStackEnd'}; %VI040210B

allControls = [buttons textLabels stackEndpoints]; %VI040210B
for i=1:length(allControls) %VI040210B
    set(gh.motorControls.(allControls{i}),'Enable','off');
end
%%%%%%%%%%%%%%%

if ~isempty(hMotor) && motorErrorPending() %VI032010A %VI100708A      
    kidControls = [findobj(gh.motorControls.figure1,'Type','uicontrol');findobj(gh.motorControls.figure1,'Type','frame');findobj(gh.motorControls.figure1,'Type','uipanel')];
    set(kidControls,'Visible','off'); 
    set(gh.motorControls.pbRecover,'Visible','on');
    %set(gh.motorControls.stRecover,'Visible','on');     
    
   	turnOffExecuteButtons;  %VI100908A
else
    drawnow expose; %VI120910A
end


