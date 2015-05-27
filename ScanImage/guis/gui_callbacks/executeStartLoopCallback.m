function executeStartLoopCallback(h)
%% function executeStartLoopCallback(h)
%
%% NOTES
%   Function significantly rewritten. See MOLD file for original version -- Vijay Iyer 1/5/11
%
%% CHANGES
%   VI051811A: Include actions that occur at start of GRAB acquisition here for LOOP acquisition; most importantly, call linTransformMirrorData() if needed -- Vijay Iyer 5/18/11
%
%% CREDITS
% Created 1/5/11, by Vijay Iyer
% Based heavily on earlier version from Cold Spring Harbor Laboratory
%% *****************************************************************************

global state gh;

state.init.eom.stackPowerScaling = ones(state.init.eom.numberOfBeams,1); %VI052909A

state.internal.whatToDo = 3; %NOTE: Try to eliminate!

val = get(h, 'String');
state.internal.loopPaused = 0;

if strcmp(val, 'LOOP')
    
    %if strcmp(get(gh.basicConfigurationGUI.figure1, 'Visible'), 'on') %VI092508A
    if state.internal.configurationChanged %VI092508A
        beep;
        setStatusString('Apply/Load Config'); %VI092508A (was 'Close Config GUI')
        return
    end
    
    %%%VI051811A%%%%
    if ~validateAcquisitionDelay() %Shows warning dialog, if needed
        return;
    end
    %%%%%%%%%%%%%%%%
    
    %TODO: Handle Cycle case for checkFileBeforeSave -- should check configuration, if any, associated with /first/ cycle index!!
    if ~savingInfoIsOK; %VI111110A %VI050409A
        return
    end
    
    % Check if file exisits %VI111110A
    overwrite = checkFileBeforeSave([state.files.fullFileName '.tif']);
    if isempty(overwrite)
        return;
    end
    
    %%%VI051811A%%%%
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
    %%%%%%%%%%%%%%%%%
    
    set(h, 'String', 'ABORT'); %VI101208A: Do this just before motor ops...so abortCurrent() will work properly if needed
    
    %MP285Clear %VI032010A: Removed %VI100608A
    
    if state.init.autoReadPMTOffsets
        startPMTOffsets;
    end
    
    set(gh.mainControls.grabOneButton, 'Visible', 'Off');
    turnOffMenus;
    
    %%%VI092508A: This shouldn't be needed, so commented out (handled by prior check)
    % 	if state.internal.configurationChanged == 1
    % 		closeConfigurationGUI;
    % 	end
    %%%%%%%%%%%%%%%
    
    resetCounters;
    resetAcqBuffer();
    
    state.internal.triggerTimeFirst =[]; %VI012510A
    state.internal.abortActionFunctions = 1;
    state.internal.stopActionFunctions = 1;
    drawnow();
    state.internal.abortActionFunctions = 0;
    state.internal.stopActionFunctions = 0;
    
    setStatusString('Starting cycle...');
    
    %Revisit the next three lines
    %stopFocus; %VI092009A
    state.internal.abort = 0;
    state.internal.currentMode = 3;
    
    %state.internal.firstTimeThroughLoop = 1; %NOTE: I don't think firstTimeThroughLoop is needed anymore
    
    state.internal.looping = 1;
    notify(state.hSI,'acquisitionStarting','LOOP');
    
    
    if state.cycle.cycleOn
        initializeCycle();
    else
        initializeLoop();
    end
    
else
    endLoopMode('abort');
end