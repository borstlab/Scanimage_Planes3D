function executeFocusCallback(h)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% executeFocusCallback(h).m******
% In Main Controls, This function is executed when the Focus or Abort button is pressed.
% It will on abort requeu the data appropriate for the configuration.
%
%% MODIFICATIONS
% 	TPMOD_1: Modified 12/31/03 Tom Pologruto - Turns off Physiology if desired before
% 	focusing so it does not trigeer an acquisition.
% 	TPMOD_2: Modified 1/5/04 Tom Pologruto - Checks flag to see if Focus
% 	shoul dbe forced to be in Frame Scan Mode
%   TPMOD_3: Modified 1/5/04 Tom Pologruto - Checks flag to see if Focus
% 	was channged to Frame Scan and should be changed back.
%   TO1904c Tim O'Connor 1/9/04 - Focus mode gets stuck in linescan when
%   toggling linescan while forceFocusFrameScan is enabled.
%   VI021908A Vijay Iyer 2/19/08 - Issue DIO trigger conditionally
%   VI041308A Vijay Iyer 4/13/08 - Don't issue DIO trigger conditionally for Focus acqs--only for Grab acqs
%   VI100608A - Vijay Iyer 10/06/08: Use MP285Clear() instead of MP285Flush()
%   VI101208A - Vijay Iyer 10/12/08: Abort FOCUS start if motor error occurs in process
%   VI010809A - Vijay Iyer 1/8/09: Use linTransformMirrorData instead of rotateAndShiftMirrorData()
%   VI011509A: (Refactoring) Remove explicit calls to setupAOData()/flushAOData(), as these are now called as part of setupDAQDevices_ConfigSpecific() -- Vijay Iyer 1/15/09
%   VI052909A: Reset stack power scaling preemptively in all cases if this function is called -- Vijay Iyer 5/29/09
%   VI090309A: (Refactoring) Defer to shared abortFocus() logic -- Vijay Iyer 9/3/09
%   VI090409A: Use new stopActionFunctions field to prevent excess callbacks -- Vijay Iyer 9/4/09
%   VI102409A: Flag condition (with warning dialog) where acquisition delay is too large for given scan speed/configuration -- Vijay Iyer 10/24/09
%   VI032010A: (Changes to use new LinearStageController class) Remove superfluous call to MP285Clear(). -- Vijay Iyer 3/20/10
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global state gh
val=get(h, 'String');
state.init.eom.stackPowerScaling = ones(state.init.eom.numberOfBeams,1); %VI052909A

if strcmp(val, 'FOCUS')
	
%	if strcmp(get(gh.basicConfigurationGUI.figure1, 'Visible'), 'on')
    if state.internal.configurationChanged
		beep;
        setStatusString('Apply/Load Config');
		return
    end
    
    %%%VI102409A%%%%%%%%
    if ~validateAcquisitionDelay() %Shows warning dialog, if needed
        return;
    end
    %%%%%%%%%%%%%%%%%%%%
	
	% start TPMOD_2 1/5/04
	if state.internal.forceFocusFrameScan
        %TO1904c
        set(gh.mainControls.cbLinescan, 'Enable', 'Off');
        
        if state.hSI.lineScanEnabled
            state.hSI.lineScanEnabled = 0;
%             updateGUIByGlobal('state.acq.linescan');
            state.acq.scanAmplitudeY= state.internal.oldAmplitude;
            updateGUIByGlobal('state.acq.scanAmplitudeY');
            setupAOData;
            %flushAOData; %VI011509A
            state.internal.forceFocusFrameScanDone=1;
        end
	end
	% end TPMOD_2 1/5/04
	
	state.internal.forceFirst=1;
	
	setStatusString('Focusing...');
	set(h, 'String', 'ABORT');
	
	set(gh.mainControls.grabOneButton, 'Visible', 'Off');
	set(gh.mainControls.startLoopButton, 'Visible', 'Off');
	if state.internal.looping
		state.internal.loopPaused=1;
	end
	turnOffMenusFocus;
	if state.init.autoReadPMTOffsets
		done=startPMTOffsets;
	end
	%TPMODPockels
	if state.internal.updatedZoomOrRot | any(state.init.eom.changed) % need to reput the data with the approprite rotation and zoom.
		%state.acq.mirrorDataOutput = rotateAndShiftMirrorData(1/state.acq.zoomFactor*state.acq.mirrorDataOutputOrg); %VI010809A
        linTransformMirrorData(); %VI010809A
		flushAOData;
		state.internal.updatedZoomOrRot=0;
	end
	
	%MP285Clear; %VI032010A: Removed
	resetCounters;
    resetAcqBuffer();
	
    notify(state.hSI,'focusStart');
    
	state.internal.abortActionFunctions=0;
    state.internal.stopActionFunctions=0; %VI090409A
	startFocus;
	%updateCurrentROI;   %TPMOD 6/18/03
	openShutter;
	state.internal.forceFirst=1;

% NOTE: For now, just set a global variable called "debugFlag" to 1, to
% enable plotting, don't comment/uncomment this stuff anymore.
% % daqdata = getDaqData(state.acq.dm, 'PockelsCell-2');
% % domain = 1000 .* (1:length(daqdata)) ./ getAOProperty(state.acq.dm, 'PockelsCell-2', 'SampleRate');
% % figure;plot(domain, daqdata);
% % title('Actual Pockels Cell Signal at time of FOCUS');
% % xlabel('Time [ms]');
% % ylabel('Voltage [V]');

	%dioTriggerConditional; %VI021908A
    dioTrigger; %VI041308A

	%*****************************************************
	%  Uncomment for benchmarking.....
	%     state.time=[];
	%     state.testtime=clock;
	%*******************************************************
	
elseif strcmp(val, 'ABORT')

    abortFocus(); %VI090309A
    
    %%%VI090309A: Removed %%%%%%%%%%%%%%%%%
    % 	state.internal.abortActionFunctions=1;
    % 	setStatusString('Aborting Focus...');
    % 	closeShutter;
    % 	set(h, 'Enable', 'off');
    % 	stopFocus(true); %Signal that this is an 'abort' operation
    % 	MP285Clear;
    %
    % 	scim_parkLaser;
    % 	flushAOData; %Refresh data after parking beam
    %
    % 	set(h, 'String', 'FOCUS');
    % 	set(h, 'Enable', 'on');
    % 	set(gh.mainControls.startLoopButton, 'Visible', 'On');
    %
    %     % start TPMOD_3 1/5/04
    % 	if state.internal.forceFocusFrameScan
    %         %TO1904c
    %         set(gh.mainControls.linescan, 'Enable', 'On');
    %
    %         if state.internal.forceFocusFrameScanDone
    %             state.internal.forceFocusFrameScanDone=0;
    %             state.acq.linescan=1;
    %             updateGUIByGlobal('state.acq.linescan');
    %             state.acq.scanAmplitudeY = 0
    %             updateGUIByGlobal('state.acq.scanAmplitudeY');
    %             setupAOData;
    %             %flushAOData; %VI011509A
    %         end
    % 	end
    % 	% end TPMOD_3 1/5/04
    %
    % 	if ~state.internal.looping
    % 		set(gh.mainControls.grabOneButton, 'Visible', 'On');
    % 		turnOnMenusFocus;
    % 	else
    % 		MP285Clear; %VI100608A
    % 		turnOffMenusFocus;
    %
    % 		resetCounters;
    % 		state.internal.abortActionFunctions=0;
    % 		setStatusString('Resuming cycle...');
    %
    % 		stopFocus;
    % 		updateGUIByGlobal('state.internal.frameCounter');
    % 		updateGUIByGlobal('state.internal.zSliceCounter');
    %
    % 		state.internal.abort=0;
    % 		state.internal.currentMode=3;
    %
    % 		mainLoop;
    % 	end
    % 	setStatusString('');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end


