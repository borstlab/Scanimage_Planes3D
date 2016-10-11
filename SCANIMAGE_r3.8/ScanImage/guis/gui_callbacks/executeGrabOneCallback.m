function executeGrabOneCallback(h,varargin)
%% function executeGrabOneCallback(h)
%
%% NOTES
%   TODO: For motor operation, verify if setting the twoStepMoveEnable and associated velocity/resolutionMode/moveMode settings is actually required -- Vijay Iyer 3/28/10
%
%% CHANGES
%   Tim O'Connor 12/16/03 :: Update the Pockels cell data, for multiple beams.
%   Tim O'Connor 2/9/04 TO2904a :: Don't access Pockels cell stuff when there's no Pockels cell.
%   Vijay Iyer 2/19/08 VI021908A :: Handle externally triggered case
%   Vijay Iyer 5/6/08 VI050608A :: Eliminate timeout error on reading position at start of stack by pausing. Remove redundant MP285Flush().
%   Vijay Iyer 5/6/08 VI050608B :: Eliminate superfluous overwrite warning message that's based on incorrect idea of checkFileBeforeSave() return value
%   Vijay Iyer 10/6/08 VI100608B :: MP285Clear() instead of MP285Flush(), after restoring its use at all.
%   Vijay Iyer 10/10/08 VI101008A :: Handle case where there's an error going home during an abort
%   Vijay Iyer 10/12/08 VI101208A :: Abort GRAB if error occurs during pre-GRAB motor operations
%   Vijay Iyer 10/13/08 VI101308A :: Handle motor velocity setting here
%   Vijay Iyer 1/8/09 VI010809A :: Use linTransformMirrorData instead of rotateAndShiftMirrorData()
%   Vijay Iyer 1/16/09 VI011609A :: Changed state.init.pockelsOn to state.init.eom.pockelsOn
%   Vijay Iyer 1/21/09 VI012109A :: msPerLine is now actually in milliseconds
%   Vijay Iyer 2/17/09 VI021709A :: Consolidate all pre-GRAB checks for conditions that should prevent GRAB from starting
%   Vijay Iyer 3/6/09 VI030609A :: Handle case of 'Focus' like GRAB acquisitions, where no file needs to be selected/saved
%   Vijay Iyer 3/24/09 VI032409A :: Provide a message box if acq delay is too large to start GRAB
%   Vijay Iyer 5/4/09 VI050409A :: Handle 'pseudofocus' case as in VI030609A, but rely on external function to determine if in that mode
%   Vijay Iyer 5/29/09 VI052909A :: Reset stack power scaling preemptively in all cases if this function is called
%   Vijay Iyer 9/3/09 VI090309A :: Handle changes to data queueing associated with new DAQmx interface. Specifically,  all data output occurs in startFocus()/startGrab(), so need not occur here.
%   Vijay Iyer 9/3/09 VI090309B :: Identify stopGrab() invoked here as an abort operation
%   Vijay Iyer 9/4/09 VI090409A :: Use new stopActionFunctions field to prevent excess callbacks
%   Vijay Iyer 10/24/09 VI102409A :: Flag condition (with warning dialog) where acquisition delay is too large for given scan speed/configuration
%   Vijay Iyer 11/01/09 VI110109A :: Set state.internal.repeatPeriod to empty when used for GRAB acquisitions. It should only contain a value for LOOP acquisitions.
%   Vijay Iyer 1/13/10 VI011310A :: Warn in cases where stack acquisition is started without either KeepAllSlicesInMemory or AutoSave enabled. This replaces previous updateKeepAllSlicesCheckMark() warning.
%   SA031210A: Replaced the MP285robustAction calls with try catch blocks
%   VI032010A: (New LinearStageController class) Set velocity/resolutionMode/moveMode to 'twoStepSlow' settings. Remove superfluous call to MP285Clear(). -- Vijay Iyer 3/20/10
%   VI032010B: (New LinearStageController class) Use motorAction() with executeGoHome() calls -- Vijay Iyer 3/20/10
%   TO091210A: BUGFIX - Multi-beam handling with PowerBox feature -- Tim O'Connor 9/12/10
%   VI092310A: Change data loss warning to reflect new data buffering scheme & user settings -- Vijay Iyer 9/23/10
%   VI100410A: Add abortAcquisitionStart/End event notification -- Vijay Iyer 10/4/10
%   VI111110A: Pseudo-focus mode has been eliminated; no longer check for this -- Vijay Iyer 11/11/10
%   DEQ112410A: Ensure that any pending move is interrupted, if possible
%   VI040511A: Open shutter immediately or with delay, depending on whether external triggering is used
%   VI052011A: Use new stepDelay INI var, to avoid appearance of jitter at start of frame following move prior to stack start, due to motor 'settling' -- Vijay Iyer 5/20/11
%   VI121211A: Call updateHeaderForAcquisition() regardless of whether pockels is used or not
%
%% CREDITS
% Written by: Thomas Pologruto and Bernardo Sabatini
% Cold Spring Harbor Labs
% January 26, 2001
%% ********************************************************************
global state gh

% determine if this function has been called from executeGrabOneStack()
if nargin > 1 && strcmp(varargin{1},'motorControlGrab')
    calledFromMotorControl = true;
else
    calledFromMotorControl = false;
end

hMotor = state.motor.hMotor; %VI032010A

% warning('Position (before): %s', num2str(state.init.eom.uncagingMapper.position));
state.internal.looping = 0;
state.init.eom.stackPowerScaling = ones(state.init.eom.numberOfBeams, 1); %VI052909A
%DEQ20110105    state.internal.repeatPeriod = []; %VI110109A

val = get(h, 'String');

if strcmp(val, 'GRAB')
    
    %needToSave = ~(state.acq.saveDuringAcquisition && state.standardMode.standardModeOn &&  ~state.standardMode.framesPerFileGUI); %VI030609A, VI050409A
    
    %%%VI021709A: Identify any conditions preventing GRAB%%%%%
    % Check for unapplied configuration changes (should not longer be possible)
    %if strcmp(get(gh.basicConfigurationGUI.figure1, 'Visible'), 'on') == 1  %VI092508A
    if state.internal.configurationChanged %VI092508A
        beep;
        setStatusString('Apply/Load Config'); %VI092508A (was 'Close Config GUI')
        return
    end
    
    
    %%%VI011310A/VI092310A: Removed %%%%%%
    %             if state.motor.motorOn && state.acq.numberOfZSlices > 1
    %                 if ~state.internal.keepAllSlicesInMemory && ~state.files.autoSave
    %                     beep;
    %                     button = questdlg('''Keep All Slices in Memory'' and ''AutoSave'' are both OFF. Data will be lost during stack acquisition. Proceed?', 'WARNING: Data May Be Lost!', 'Yes', 'No', 'No');
    %                     if strcmpi(button,'No')
    %                         return;
    %                     end
    %                 end
    %             end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%VI092310A%%%
    if ~state.files.autoSave && state.internal.dataBufferingLoss
        button = questdlg('With current data buffering User Settings, some frames/slices will be unavailable to save following acquisition. Proceed?', 'WARNING: Data Loss!', 'Yes', 'No', 'No');
        if strcmpi(button,'No')
            return;
        end
    end
    %%%%%%%%%%%%%%
    
    %%%VI102409A%%%%%%%%
    if ~validateAcquisitionDelay() %Shows warning dialog, if needed
        return;
    end
    %%%%%%%%%%%%%%%%%%%%
    
    %Ensure file is specified
    ok = savingInfoIsOK;
    if ok == 0
        return;
    end
    
    % Check if file exisits (VI111110A)
    overwrite = checkFileBeforeSave([state.files.fullFileName '.tif']);
    if isempty(overwrite)
        return;
        %%%%%% VI050608B%%%%%%%%%%
        %elseif ~overwrite
        %      %TPMOD 2/6/02
        %      if state.files.autoSave || state.acq.saveDuringAcquisition
        %	    disp('Overwriting Data!!');
        %      end
        %%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    
    % Check if acq delay is too high
    [startColumn endColumn] = determineAcqColumns();
    if endColumn > state.internal.samplesPerLine && state.internal.numberOfStripes > 1
        msgbox('Acquisition delay is too high. Acquisition not possible. Either reduce acquisition delay or disable image striping.', 'Acq Delay Too High','warn','modal'); %VI032409A
        setStatusString('Acq Delay Too High!');
        return;
    end
    
    if state.internal.updatedZoomOrRot % need to reput the data with the approprite rotation and zoom.
        %state.acq.mirrorDataOutput = rotateAndShiftMirrorData(1 / state.acq.zoomFactor * state.acq.mirrorDataOutputOrg); %VI010809A
        linTransformMirrorData(); %VI010809A
        flushAOData;
        state.internal.updatedZoomOrRot = 0;
    end
    
    %%%VI090309A: Removed %%%%%%%%%%%%%%%%%%
    %             %Update the Pockels cell signal(s) if necessary. 12/16/03
    %             %Only do this if the pockels cell code is active. - TO2904a
    %             if state.init.eom.pockelsOn %VI011609A
    %                 for beamCounter = 1 : state.init.eom.numberOfBeams
    %                     if state.init.eom.changed(beamCounter)
    %                         %                     putDaqData(state.acq.dm, state.init.eom.pockelsCellNames{beamCounter}, ...
    %                         %                         repmat(makePockelsCellDataOutput(beamCounter), [state.acq.numberOfFrames 1]) ...
    %                         %                         );
    %                         %The data is already replicated out to the correct number of frames.
    %                         putDaqData(state.acq.dm, state.init.eom.pockelsCellNames{beamCounter}, ...
    %                             makePockelsCellDataOutput(beamCounter));
    %                     end
    %                 end
    %             end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % 			startZoom;
    if state.init.autoReadPMTOffsets
        startPMTOffsets;
    end
    
    %%%%Motor preparations
    if state.motor.motorOn && state.acq.numberOfZSlices > 1
        %MP285Clear; %VI032010A:Removed %VI050608A, VI100608A
        %pause(.5); %VI032010A %VI050608A
        
        motorGetPosition();
        %               if MP285RobustAction(@updateMotorPosition,'record position at start of stack', mfilename) %VI101508A %SA031210
        %                 try
        %                     updateMotorPosition();
        %                 catch ME
        %                     abortCurrent();
        %                     ME.rethrow();
        %                 end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        state.internal.initialMotorPosition = state.motor.lastPositionRead;
        
        %if MP285RobustAction(@()MP285SetVelocity(state.motor.velocitySlow,1), 'set motor velocity at start of stack', mfilename) %VI101508A %SA031210
        %motorAction(@()set(hMotor,'twoStepMoveEnable',false));
        
        % if 'stack position is start center', calculate the z-offset
        if state.acq.stackCentered && ~calledFromMotorControl
            state.acq.stackCenteredOffset = floor((state.acq.numberOfZSlices*state.acq.zStepSize)/2.0);
            
            if state.motor.motorZEnable && state.motor.dimensionsXYZZ
                motorSetPositionAbsolute(state.internal.initialMotorPosition - [0 0 0 state.acq.stackCenteredOffset],'verify');
            else            
                motorSetPositionAbsolute(state.internal.initialMotorPosition(1:3) - [0 0 state.acq.stackCenteredOffset],'verify');
            end
            pause(state.motor.stepDelay); %VI052011A
        elseif ~state.acq.stackCentered
            state.acq.stackCenteredOffset = 0;
        end
        
    else
        state.internal.initialMotorPosition = [];
    end
    %%%%%%%%%%%
    
    set(h, 'String', 'ABORT');
    
    setStatusString('Acquiring Grab...');
    set([gh.mainControls.focusButton gh.mainControls.startLoopButton], 'Visible', 'Off');
    turnOffMenus;
    
    resetCounters;
    resetAcqBuffer();
    
    state.internal.triggerTimeFirst =[]; %VI012510A
    state.internal.abortActionFunctions = 0;
    state.internal.stopActionFunctions = 0;
    
    updateGUIByGlobal('state.internal.frameCounter');
    updateGUIByGlobal('state.internal.zSliceCounter');
    
    %updateCurrentROI;   %TPMOD 6/18/03
    
    try
        if state.init.eom.pockelsOn %VI011609A
            for i = 1 : state.init.eom.numberOfBeams
                if length(state.init.eom.showBoxArray) < i %TO091210A
                    continue;
                end
                if state.init.eom.showBoxArray(i)
                    state.init.eom.powerBoxWidthsInMs(i) = round(100 * state.init.eom.powerBoxNormCoords(i, 3) ...
                        * state.acq.msPerLine / state.acq.pixelsPerLine) / 100; %VI012109A
                else
                    state.init.eom.powerBoxWidthsInMs(i) = 0;
                end
            end
            if length(state.init.eom.showBoxArray) < state.init.eom.numberOfBeams
                state.init.eom.showBoxArray(state.init.eom.numberOfBeams) = 0;
            end
            %                     if length(state.init.eom.uncagingPulseImporter.enabled) < state.init.eom.numberOfBeams
            %                         state.init.eom.uncagingPulseImporter.enabled(state.init.eom.numberOfBeams) = 0;
            %                     end
            %updateHeaderForAcquisition; %VI121211A: Relocated
            
        end
    catch
        warning(sprintf('Error in saving Pockels Cell data to header (executeGrabOneCallback): %s\n', lasterr));
    end
    
    updateHeaderForAcquisition(); %VI121211A
    
    notify(state.hSI,'acquisitionStarting','GRAB');
    
    startGrab;
    if state.shutter.shutterDelay == 0
        openShutter(~state.acq.externallyTriggered); %VI040511A
    else
        state.shutter.shutterOpen = 0;
    end
    
    % NOTE: For now, just set a global variable called "debugFlag" to 1, to
    % enable plotting, don't comment/uncomment this stuff anymore.
    % daqdata = getDaqData(state.acq.dm, 'PockelsCell-2');
    % domain = 1000 .* (1:length(daqdata)) ./ getAOProperty(state.acq.dm, 'PockelsCell-2', 'SampleRate');
    % figure;plot(domain, daqdata);
    % title('Actual Pockels Cell Signal at time of Grab');
    % xlabel('Time [ms]');
    % ylabel('Voltage [V]');
    dioTriggerConditional; %VI021908A
    
elseif strcmp(val, 'ABORT')
    %TPMOD 7/7/03....
    if state.internal.roiCycleExecuting
        abortROICycle;
        return
    end
    
    state.internal.abortActionFunctions = 1;
    
    notify(state.hSI,'abortAcquisitionStart'); %VI100410A
    
    closeShutter;
    stopGrab(true); %VI090309A: Identify as an abort operation
    
    setStatusString('Aborting...');
    set(h, 'Enable', 'off');
    
    scim_parkLaser;
    flushAOData;
    setImagesToWhole();
    
    %if ~executeGoHome  %VI032010B %VI101008A: Only restore Grab button if no MP285 error caused (or pre-existing)
    motorAbort(); %DEQ112410
    motorGoHome(); %Goes to stack home, if any %VI032010B
    set(h, 'Enable', 'on');
    %end
    
    setStatusString('Aborted Grab');
    set(h, 'String', 'GRAB');
    set([gh.mainControls.focusButton gh.mainControls.startLoopButton], 'Visible', 'On');
    turnOnMenus;
    
    notify(state.hSI,'abortAcquisitionEnd'); %VI100410A
else
    disp('executeGrabOneCallback: Grab One button is in unknown state'); 	% BSMOD - error checking
end

return;