%% function turnOnMotorButtons
%% CHANGES 
% VI100708A: Handle error condition case handling -- Vijay Iyer 10/07/08
% VI100808A: Handle button disabling programatically. Add Grab and stack start/stop buttons to list. -- Vijay Iyer 10/08/08
% VI103008A: Add all the motorControls controls to the list -- Vijay Iyer 10/30/08
% VI032010A: (Changes to use new LinearStageController class) Use motor object's error condition directly -- Vijay Iyer 3/20/10
% VI040210A: Add checkbox motorControls controls to list of  'buttons' -- Vijay Iyer 4/2/10
% VI040210B: Add all text labels and stack endpoint edit text controls to list of controls to re-enable (using 'inactive' for latter) -- Vijay Iyer 4/2/10
% VI051211A: Allow controls to be excluded from the visible motor controls -- Vijay Iyer 4/21/11
%% ************************************************
function turnOnMotorButtons
global state gh

hMotor = state.motor.hMotor; %VI032010

if ~hMotor.lscErrPending && (~state.motor.motorZOn || ~state.motor.hMotorZ.lscErrPending)  %VI032010A  %VI100708A    
    kidControls = [findobj(gh.motorControls.figure1,'Type','uicontrol');findobj(gh.motorControls.figure1,'Type','frame');findobj(gh.motorControls.figure1,'Type','uipanel')];
    set(setdiff(kidControls,state.motor.excludeControls),'Visible','on'); %VI051211A
    set(gh.motorControls.pbRecover,'Visible','off');
    set(gh.motorControls.stRecover,'Visible','off');
   
    turnOnExecuteButtons; %VI100708A (Execute buttons are turned off during an error)
    
    %%%VI100808A, VI103008A%%%%%%%%
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

    enableControls = [buttons textLabels]; %VI040210B
    for i=1:length(enableControls)
        set(gh.motorControls.(enableControls{i}),'Enable','on'); %VI040210B
    end
    
    %%%VI040210B
    for i=1:length(stackEndpoints)
        set(gh.motorControls.(stackEndpoints{i}),'Enable','inactive');
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
else
    
    fprintf(2,'WARNING (%s): Cannot restore motor control buttons while in MP285 error condition\n',mfilename);
end

