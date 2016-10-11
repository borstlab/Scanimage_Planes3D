function genericKeyPressFcn(hObject,eventdata)
%% function genericKeyPressFcn

% genericKeyPressFcn.m*****
% Function that looks at the last key pressed and executes an appropriate function.
% First gets all the current character from all the GUIs
%
% Written By: Thomas Pologruto and Bernardo Sabatini
% Cold Spring Harbor Labs
% April 2, 2003
%% NOTES
%    Currently, this function is bound to the genericKeyPressFcn of the Main Controls, Standard Mode, ImageGUI, MotorGUI, powerControl, the Configuration GUI, and the image figures
%    The binding occurs manually, rather than through GUIDE, allowing the specification of a function handle, rather than a named function. This allows 'eventdata' to be used.
%
%% CHANGES
%   VI043008A Vijay Iyer 4/30/08 -- Change fast config keys to QWERTY
%   VI043008B Vijay Iyer 4/30/08 -- Enforce requirement for 'CTL+' modifier for fast configs, where specified
%   VI043008C Vijay Iyer 4/30/08 -- Don't do rotations when in bidirectional scanning mode
%   VI043008D Vijay Iyer 4/30/08 -- Use the (new) eventdata structure to more robustly extract the pressed key (this requires use of function handle to specify callback)
%   VI070208A Vijay Iyer 7/02/08 -- Return focus to window from which this callback was spawned
%   VI070308A Vijay Iyer 7/03/08 -- Screen out modifier-only key press events--this callback will now be used for keyPressFcn events, rather than keyReleaseFcn events.
%   VI090908A Vijay Iyer 9/09/08 -- Override VI04308C and handle bidscanning case in setRotation() helper
%   VI090908B Vijay Iyer 9/09/08 -- Handle all cases via a single switch statement and use eventdata.Key naming, rather than character or character code, to determine hotkey (this works for case where CTL is added or not)
%   VI090908C Vijay Iyer 9/09/08 -- Re-map fast config keys to F1-F6; remap zoom to I/O; remap rotate to <,> ; remap Focus/Grab/Loop to f,g,l
%   VI090908D Vijay Iyer 9/09/08 -- Remove 'tab' mapping to Snapshot. Not documented or needed. Tab key by default maps to change focus between controls in windo
%   VI110808A Vijay Iyer 11/08/08 -- 'Safe' hotkey feature (i.e. requiring CTL+ modifier) now separated between Fast Config and other keyboard shortcuts -- Vijay Iyer 11/08/08
%   VI111708A Vijay Iyer 11/17/08 -- Handle case (bug) where eventdata does not get passed. These 'non-events' shouldn't have any response
%   VI111708B Vijay Iyer 11/17/08 -- Change hotkeys for positions 1-7 to require CTL, rather than SHIFT, and also tied to hotkey-needs-CTL setting
%   VI120108A Vijay Iyer 12/01/08 -- Handle 'save as' hotkey the same as all the other hotkeys...
%   VI010909A Vijay Iyer 01/09/09 -- Use updateZoom and caching of previous value when changing zoom value
%   VI032310A Vijay Iyer 03/23/10 -- Refactoring to use new LinearStageController class
%   VI060610A Vijay Iyer 6/6/10 -- BUGFIX: Make correct call to motorSetRelativeOrigin() on '0' key press
%   VI060610B Vijay Iyer 6/6/10 -- BUGFIX: Ensure the 'z' goto-zero operation only occurs if a relative origin has been defined (for all dimensions)
%   VI092910A Vijay Iyer 9/29/10 -- Consolidate F1-F6 FastConfig handling; interpret shift modifier when selecting FastConfigs (F1-F6) -- Vijay Iyer 9/29/10
%
%% ************************************************
global state gh

% if state.internal.ignoreKeyPress
%     return
% end

%Get figure that spawned this event, so focus can be returned to it
hFig = ancestor(gcbo,'figure'); %VI070208A

buttonHandles=[gh.mainControls.grabOneButton gh.mainControls.focusButton gh.mainControls.startLoopButton];

%val = double(get(gcbo,'CurrentCharacter'));
%val = double(eventdata.Key); %VI043008D

if isempty(eventdata) %VI111708A
    return
end

val = eventdata.Key; %VI090908B

%Ignore modifier if not accompanied by another key/character
if isempty(val) || any(ismember({'control' 'alt' 'shift'},{eventdata.Key})) % || (state.init.hotKeysNeedCtl && ~any(ismember({'control' 'shift'} ,eventdata.Modifier))) %VI043008B, VI070308A, VI090908D, VI110808A
    return
end

%%%110808A: If no control modifier is present, determine whether to proceed
if ~any(ismember({'control'} ,eventdata.Modifier))
    isFastConfig = any(ismember({eventdata.Key},{'f1' 'f2' 'f3' 'f4' 'f5' 'f6'}));
    
    if state.init.hotKeysNeedCtl && ~isFastConfig
        return;
    elseif state.init.fastConfigHotKeysNeedCtl && isFastConfig
        return;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     

shiftMod = ismember('shift', eventdata.Modifier); %VI092910A

%%%%VI090908B -- Removed special case handling for zoom, rotation, shift, and motor position controls here -- see end of file
state.internal.ignoreKeyPress=1;
switch val
    case {'1' '2' '3' '4' '5' '6' '7'}
        %if ismember('shift',eventdata.Modifier) %VI111708B
        p = str2num(val); %VI090908B
        %p=find(val==[33 64 35 36 37 94 38]); % Shifted Numbers from 1 to 7
        turnOffMotorButtons;
        definePosition(p);
        turnOnMotorButtons;
        %%%VI111708B: Don't require shift anymore%%%%%
        % else %as if key was never hit
        %             state.internal.ignoreKeyPress=0;
        %             return;
        % end
        %%%%%%%%%%%%%%%%%%
    case '0'
        %if ismember('shift',eventdata.Modifier) %VI111708B
        setStatusString('Defining (0,0,0)...');        
        motorSetRelativeOrigin([1 1 1 1]); %VI060610A %VI032310A
        setStatusString('Defined (0,0,0)');

        %%%VI032310A: Removed%%%%%%%
        %         turnOffMotorButtons;
        %         updateMotorPosition(0);
        %         state.motor.offsetX=state.motor.absXPosition;
        %         state.motor.offsetY=state.motor.absYPosition;
        %         state.motor.offsetZ=state.motor.absZPosition;
        %         updateRelativeMotorPosition;
        %         turnOnMotorButtons;
        %         setStatusString('');
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%VI111708B: Don't require shift anymore%%%%%
        % else %as if key was never hit
        %   state.internal.ignoreKeyPress=0;
        %   return;
        % end
        %%%%%%%%%%%%%%%%%%%%%%%
%     case 'tab' % 'tab' take snapshot (VI090908B: was 9)
%         if all(strcmpi(get(buttonHandles,'Visible'),'on'))
%             snapShot(state.acq.numberOfFramesSnap);
%         end

    case {'f1' 'f2' 'f3' 'f4' 'f5' 'f6'} %load fast configuration  %VI092910A VI043008A, VI090908B, VI090908C)
        if all(strcmpi(get(buttonHandles,'Visible'),'on')) 
            configNum = str2num(val(2:end)); %strip off the 'f'
            loadFastConfig(configNum,shiftMod); %VI092910A: suppress auto-start if shift-modifier is active %
        end
        
    case 'z' % z Go to zero motor position.... (VI090908B: was 122)
        
        %TODO: Handle behavior (if any) related to secondary Z motor
        
        %%%VI060610A%%%
        if ~all(state.motor.hMotor.zeroSoftFlag)
            fprintf(2,'WARNING: Relative origin must be defined (for all dimensions) in order to move to relative origin\n');
            return;
        end                 
        %%%%%%%%%%%%%%%

        secZEnable = state.motor.motorZEnable && ...
            state.motor.dimensionsXYZZ && state.motor.hMotor.zeroSoftFlag(3);

        turnOffMotorButtons;        
        %gotoZero; %VI032310A: Removed
        if secZEnable
            setStatusString('Moving to (0,0,0,0)'); 
        else
            setStatusString('Moving to (0,0,0)'); %VI032310A
        end
        motorSetPositionRelative([0 0 0 0],'verify'); %VI052010A %VI032310A
        turnOnMotorButtons;
    case 'a' %a abort.... (VI090908B: was 97)
        abortCurrent;
    case 'f' %f Focus (VI090908B/C: changed 48('0') to 'f')
        if all(strcmpi(get(buttonHandles,'Visible'),'on')) | all(strcmpi(get(buttonHandles([1 3]),'Visible'),'off'))
            figure(gh.mainControls.figure1);
            state.internal.whatToDo=1;
            executeFocusCallback(gh.mainControls.focusButton);
        end
    case 'g' %g Grab (VI090908B/C: changed 46('.') to 'g')
        if all(strcmpi(get(buttonHandles,'Visible'),'on')) | all(strcmpi(get(buttonHandles([2 3]),'Visible'),'off'))
            figure(gh.mainControls.figure1);
            state.internal.whatToDo=2;
            executeGrabOneCallback(gh.mainControls.grabOneButton);
        end
    case 'l' %l Loop (VI090908B/C: changed 43('+') to 'l')
        if all(strcmpi(get(buttonHandles,'Visible'),'on')) | all(strcmpi(get(buttonHandles([1]),'Visible'),'off'))
            figure(gh.mainControls.figure1);
            executeStartLoopCallback(gh.mainControls.startLoopButton);
        end
    case 'p' % p update motor position.... (VI090908B: was 112)
        turnOffMotorButtons;
        motorGetPosition(); %VI032310A
        turnOnMotorButtons;
    case 's' % s Save last acquisition As... (VI120108A: was 19)
        saveLastAcquisitionAs;
        
    %Handle shift cases  (VI090908B/C)
    case 'uparrow' 
        mainControls('up_Callback',gh.mainControls.up);
    case 'downarrow' 
        mainControls('down_Callback',gh.mainControls.down);
    case 'leftarrow'
        mainControls('left_Callback',gh.mainControls.left);
    case 'rightarrow'        
        mainControls('right_Callback',gh.mainControls.right);
        
    %Handle zoom in/out cases  (VI090908B/C)
    case 'i'
        setZoomValue(state.acq.zoomFactor + 2);
    case 'o'
        setZoomValue(state.acq.zoomFactor - 2);
        
    %Handle rotate left/right cases  
    case 'comma' %the '<' key - on US keyboards (VI090908C)
        setRotation(-5);
    case 'period' %the '>' key - on US keyboards (VI090908C)
        setRotation(5);                      
    case 'space'
%         state.acq.linescan=1-state.acq.linescan;
		state.hSI.lineScanEnabled = 1 - state.hSI.lineScanEnabled;
        updateGUIByGlobal('state.acq.linescan');
        mainControls('linescan_Callback',gh.mainControls.linescan);
end
state.internal.ignoreKeyPress=0;
figure(hFig); %VI070208A

%***************************************************************
% function setZoom(direction)
% global state gh
% 
% oldZoomFactor = state.acq.zoomFactor; %VI010909A: Cache old zoom value
% state.acq.zoomFactor=round(state.acq.zoomFactor+direction);
% if state.acq.zoomFactor<1
%     state.acq.zoomFactor=1;
% end
% %%%VI010909A: Flag if zoom change may have altered the fill fraction (line period)
% if state.acq.zoomFactor >= state.acq.baseZoomFactor && oldZoomFactor < state.acq.baseZoomFactor
%     state.internal.fillFracChange = 1;
% elseif state.acq.zoomFactor < state.acq.baseZoomFactor
%     state.internal.fillFracChange = 1;
% else
%     state.internal.fillFracChange = 0;
% end
% %%%%%%%%%%%
% updateZoom(); %VI010909A

%%%VI010909A: Removed %%%%%%%%%%%%%
% updateGUIByGlobal('state.acq.zoomFactor');
% zoomstr= num2str(state.acq.zoomFactor);
% if length(zoomstr)==1
%     zoomstr=['00' zoomstr];
% elseif length(zoomstr)==2
%     zoomstr=['0' zoomstr];
% elseif length(zoomstr)>3
%     zoomstr=zoomstr(1:3);
% end
% state.acq.zoomhundreds=str2num(zoomstr(1));
% state.acq.zoomtens=str2num(zoomstr(2));
% state.acq.zoomones=str2num(zoomstr(3));
% updateGUIByGlobal('state.acq.zoomhundreds');
% updateGUIByGlobal('state.acq.zoomtens');
% updateGUIByGlobal('state.acq.zoomones');
% mainControls('zoomones_Callback',gh.mainControls.zoomones);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%***************************************************************
function setRotation(direction)
global state gh
if ~state.acq.bidirectionalScan
    state.acq.scanRotation=state.acq.scanRotation+direction;
    updateGUIByGlobal('state.acq.scanRotation');
    mainControls('genericZoomRot_Callback',gh.mainControls.zoomones);
end

%%%VI090908B: Original section below removed; logic moved into shared switch statement
% %Motor positions.....
% if any(val==[33 64 35 36 37 94 38])
%     state.internal.ignoreKeyPress=1;
%     p=find(val==[33 64 35 36 37 94 38]); % Shifted Numbers from 1 to 7
%     turnOffMotorButtons;
%     definePosition(p);
%     turnOnMotorButtons;
%     state.internal.ignoreKeyPress=0;
%     figure(hFig); %VI070208A
%     return
% end
% 
% %Move image with arrow keys.....
% if val>=28 & val<=31 % arrow keys
%     state.internal.ignoreKeyPress=1;
%     if val==30
%         mainControls('up_Callback',gh.mainControls.up);
%     elseif val==31
%         mainControls('down_Callback',gh.mainControls.down);
%     elseif val==28
%         mainControls('left_Callback',gh.mainControls.left);
%     elseif val==29
%         mainControls('right_Callback',gh.mainControls.right);
%     end
%     state.internal.ignoreKeyPress=0;
%     figure(hFig); %VI070208A
%     return
% end
% 
% %Zooming and rotation.....
% if any(val==[32 49 50 52 53]) % Numbers 1,2 4,5
%     state.internal.ignoreKeyPress=1;
%     if val==32  %SpaceBar toggle linescan
%         state.acq.linescan=1-state.acq.linescan;
%         updateGUIByGlobal('state.acq.linescan');
%         mainControls('linescan_Callback',gh.mainControls.linescan);
%     elseif val==49  %1 zoom out
%         setZoom(-2);
%     elseif val==50  %2 zoom in
%        setZoom(2);
%     elseif val==52 && ~state.acq.bidirectionalScan %4 rotate left  (VI043008C)
%         setRotation(-5);
%     elseif val==53 && ~state.acq.bidirectionalScan %5 rotate right (VI043008C)
%         setRotation(5);
%     end
%     state.internal.ignoreKeyPress=0;
%     figure(hFig); %VI070208A
%     return
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%