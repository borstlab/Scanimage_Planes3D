function endAcquisition(abort)
%% function endAcquisition(abort)
% Function called when state.acq.numberOfFrames have been acquired (as part of a movie or for averaging)
% This function either handles clean up (closing file and stopping the GRAB/LOOP acquisition), or it prepares for next acquisition in stack or LOOP (which is the next cycle position, in CycleControls mode)
%
%% SYNTAX
%   abort: (OPTIONAL) Logical value indicating, if true, that this is an 'abort' operation. If empty/omitted, false is assumed
%
%% CHANGES
% VI031108A Vijay Iyer 3/11/08 - Don't auto-save if data's been saved during acquisition
% VI082208A Vijay Iyer 8/22/08 - Close tifstream for saveDuringAcquisition mode
% VI093008A Vijay Iyer 9/30/08 - Abort stack collection if movement failed
% VI100608A Vijay Iyer 10/06/08 - Handle MP-285 error conditions smartly 
% VI101008A Vijay Iyer 10/10/08 - Handle MP-285 failure to return home
% VI101508A Vijay Iyer 10/15/08 - Use new MP285RobustAction for executeGoHome and MP285FinishMove actions
% VI090309A Vijay Iyer 9/3/09 - With new DAQmx interface, use flushAOData() now to refresh data following beam park
% VI090909A Vijay Iyer 9/9/09 - Completion of VI090309A
% VI091909A Vijay Iyer 9/19/09 - Handle the case where endAcquisition() is called as an abort operation
% VI092009A Vijay Iyer 9/20/09 - Cut & paste code snippet to newly created closeTifStream() to allow code sharing
% VI103009A Vijay Iyer 10/30/09 - If looping in Cycle mode, don't return to stack home here -- leave this to mainLoop()
% SA031210A: Replace MP285RobustAction() calls with try/catch block for now -- Salvador Aguinaga 03/12/10
% VI032010A: Use motorAction now, in lieu of original MP285RobustAction and SA031210A. Call motorStartMove(), which replaces startMoveStackFocus(). -- Vijay Iyer 3/20/10
% VI032010B: Use new motorFinishMove(), replacing MP285FinishMove(), as part of LinearStageControllerBasic class refactoring -- Vijay Iyer 3/20/10
% VI091610A: motorGoHome() is a 'macro', not an 'action'; shouldn't use motorAction() wrapper  -- Vijay Iyer 9/16/10
% VI092210A: If autoSave is on, we now always save during acquisition -- Vijay Iyer 9/22/10
% VI100410A: Replace callUserFunction() calls; use new user function scheme -- Vijay Iyer 10/4/10
% VI112310A: Restore call to writeMaxData() here -- Vijay Iyer 11/23/10
% VI012511A: The zSliceCounter is now 1-based -- Vijay Iyer 1/25/10
% VI040511A: Wait for shutter to open after sending electrical command -- Vijay Iyer 4/5/11
% VI051711A: Remove deprecated roiCycle code reference -- Vijay Iyer 5/17/11 
% VI051811A: Support new stepDelay INI var, to avoid appearance of jitter at start of frame following slice moves due to motor 'settling' -- Vijay Iyer 5/18/11
%
%% CREDITS
% Written By: Thomas Pologruto and Bernardo Sabatini
% Cold Spring Harbor Labs
% March 2, 2001
%% ******************************************************************
global state gh

if nargin < 1
    abort = false;
end


% if user has aborted, then return
if state.internal.abortActionFunctions
    abortInActionFunction;
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate MAx projections if necessary%%%%%%%%%%%%%%%%%%
%%TPMOD for roiCycles....7/21/03 % Code for displaying Max Projections
calculateMaxProjections;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now setup for another pass if possible....
% if state.internal.zSliceCounter == state.acq.numberOfZSlices %VI012511A
if state.internal.zSliceCounter == state.acq.numberOfZSlices - 1
    % Done Acquisition since there are no more stacks....
    stopGrab(abort); %VI091909A
    
    %TPMOD for SnapShot Mode....6/2/03
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if state.internal.snapping
        doSnapShot;
		notify(state.hSI,'acquisitionDone');
        return
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%VI092210A: REMOVED%%%%%%%%
    %     %Save the data to disk....
    %     if state.files.autoSave && ~state.acq.saveDuringAcquisition	% BSMOD - Check status of autoSave option  % VI031108A
    %         status=state.internal.statusString;
    %         setStatusString('Writing data...');
    %         writeData;
    %         writeMaxData;
    %         setStatusString(status);
    %         state.files.fileCounter=state.files.fileCounter+1;
    %         updateGUIByGlobal('state.files.fileCounter');
    %         updateFullFileName(0);
    %     elseif state.acq.saveDuringAcquisition && ~isempty(state.files.tifStream) %VI031108A, VI082208A
    %         closeTIFStream(); %VI092009A
    %     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    %%%VI092210A%%%%
    %Handle file saving
    if state.files.autoSave && ~isempty(state.files.tifStream)                
        %%%VI112310A
        if any(state.acq.maxImage)
            writeMaxData();
        end    
        
        closeTIFStream(); %VI092009A %This call increments state.files.fileCounter
    end
    %%%%%%%%%%%%%%%%
    
    %VI100410A:Removed %TO051804a
    %callUserFunction;
    
    %%%VI051711A: Removed%%%%
    %TPMOD for roiCycles....7/10/03
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     if state.internal.roiCycleExecuting % Doing user defined cycle...
    %         loopROICycle;
    %         return
    %     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    scim_parkLaser;
    %putDataGrab; %VI090309A
    flushAOData(); %VI090309A
    
    %%%VI012511A: Removed%%%%
    %     state.internal.zSliceCounter = state.internal.zSliceCounter + 1;
    %     updateGUIByGlobal('state.internal.zSliceCounter');
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if state.acq.numberOfZSlices > 1 %VI012511A  
        motorFinishMove(); %VI032010B
        
        %%%VI032010B: Removed%%%%%%%%%%
        %         if MP285FinishMove(1)
        %             MP285Recover; %Interrupt the move..and get the position
        %             if state.motor.errorCond
        %                 fprintf(2,'ERROR (%s): Unable to verify correct completion of stack motion', mfilename);
        %             else
        %                 MP285FinishMove(1); %check the position and flag if it's not as expected
        %             end
        %         end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        if ~state.internal.looping || ~state.cycle.cycleOn %VI103009A
            %if MP285RobustAction(@executeGoHome, 'return home upon stack completion', mfilename) %SA031210A %VI101508A
            %motorAction(@motorGoHome, 'Return Home upon Stack Completion'); %VI091610A: Removed %VI032010A
            motorGoHome(); %Go to stack home %VI091610A
        end
        
        %VI010610A: Reset max power display -- needed in case P vs Z was enabled during just-completed stack acquisition
        if state.init.eom.pockelsOn
            updateMaxPowerDisplay(state.init.eom.beamMenu);
        end
            
	end		
	
	state.internal.zSliceCounter = state.internal.zSliceCounter + 1;
    updateGUIByGlobal('state.internal.zSliceCounter');
    
    notify(state.hSI,'acquisitionDone'); %VI100410A
    
    if state.internal.looping==1
        setStatusString('Resuming Loop....');
        iterateLoop();
    else
        setImagesToWhole();
        
        setStatusString('Ending Grab...');
        set(gh.mainControls.focusButton, 'Visible', 'On');
        set(gh.mainControls.startLoopButton, 'Visible', 'On');
        set(gh.mainControls.grabOneButton, 'String', 'GRAB');
        set(gh.mainControls.grabOneButton, 'Visible', 'On');
        turnOnMenus;
        setStatusString('');
    end
elseif state.internal.zSliceCounter < state.acq.numberOfZSlices %VI012511A 
    % Between Acquisitions or ZSlices
    setStatusString('Next Slice...');
    
    %%%VI092210A: REMOVED %%
    %     if state.files.autoSave		% BSMOD - Check status of autoSave option
    %         setStatusString('Writing data...');
    %         writeData;
    %     end
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    %VI100410A:Removed %TO051804a
    %callUserFunction;
    
    state.internal.zSliceCounter = state.internal.zSliceCounter + 1;
    updateGUIByGlobal('state.internal.zSliceCounter');
    
%     state.internal.frameCounter = 1;
	state.internal.frameCounter = 0;
    updateGUIByGlobal('state.internal.frameCounter');
    
    setStatusString('Acquiring...');
    
    %putDataGrab; %VI090909A
    flushAOData(); %VI090909A
    
    %if MP285FinishMove(0)	% check that movement completed (e.g. a CR was sent back), but don't verify position. This proved too unreliable so far -- Vijay Iyer 10/06/08
    motorFinishMove(); %VI032010B: Check that movement completed, but don't verify that position is accurate
    %return;
    %%%VI032010B: Removed %%%%%%%%%
    %     if MP285FinishMove(0)	% check that movement completed (e.g. a CR was sent back), but don't verify position. This proved too unreliable so far -- Vijay Iyer 10/06/08
    %         abortCurrent;  %VI093008A
    %         return;
    %     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    most.idioms.pauseTight(state.motor.stepDelay); %VI051811A
        
    notify(state.hSI,'sliceDone'); %VI100410A
        
    if (strcmp(get(gh.mainControls.grabOneButton, 'String'), 'GRAB') ...
            & strcmp(get(gh.mainControls.grabOneButton, 'Visible'),'on'))
        set(gh.mainControls.grabOneButton, 'enable', 'off');
        set(gh.mainControls.grabOneButton, 'enable', 'on');
    elseif (strcmp(get(gh.mainControls.startLoopButton, 'String'), 'LOOP') ...
            & strcmp(get(gh.mainControls.startLoopButton, 'Visible'),'on'))
        set(gh.mainControls.startLoopButton, 'enable', 'off');
        state.internal.abort=1;
        set(gh.mainControls.startLoopButton, 'enable', 'on');
    else
        try; startGrab; catch; end
        openShutter(true); %VI040511A
        dioTrigger;
    end
end

%%%%VI100410A:Removed%%%%%%%%%%%%%%%%%
% %TO051804a - Call user function after saving data to a file.
% %            Also, wrap with a try/catch.
% function callUserFunction
% global state;
% %%%%%%%%%%%%%%%%%%User Function Call%%%%%%%%%%%%%%%%% TPMOD
% try
%     if state.userFcnGUI.UserFcnOn
%         if (state.internal.snapping & state.acq.execUserFcnOnSnap) | ~state.internal.snapping
%             executeUserFcn;
%         end        
%     end
% catch
%     warning('Error executing UserFunction: %s', lasterr);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
