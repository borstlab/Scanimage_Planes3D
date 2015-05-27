function startGrab
%% function startGrab
% Starts the DAQ objects for focusing (hAO, hAI).
%
%% NOTES
%   Function rewritten completely from scratch. Previous version can be seen in .MOLD file. -- Vijay Iyer 9/03/09
%
%% CHANGES
% 3/07/08 Vijay Iyer (VI030708A): Handle case where saving during acquisition is enabled
% 4/13/08 Vijay Iyer (VI041308A): Handle trigger properties based on External trigger toggle-button
% 6/23/08 Vijay Iyer (VI062308A): Update header info to correctly reflect the framesPerFile value
% 6/23/08 Vijay Iyer (VI062308B): Set framesPerFile to inf when all frames will be saved to one file
% 8/12/08 Vijay Iyer (VI081208A): Don't use manual BufferingConfig
% 8/13/08 Vijay Iyer (VI081308A): Special Pockels features only apply if Pockels is on 
% 8/21/08 Vijay Iyer (VI082108A): Set up @tifstream object if using saveDuringAcquisition
% 8/21/08 Vijay Iyer (VI082108B): Don't chunk files if frames/file equal # of frames
% 12/02/08 Vijay Iyer (VI120208A): Handle more gracefully the case where next file to create already existed
% 3/06/09 Vijay Iyer (VI030609A): Handle case of 'Focus'-like GRAB where no save is needed
% 9/19/09 Vijay Iyer (VI091909A): Use (new) armTrigger() and start() on all acquisition tasks en masse
% 9/20/09 Vijay Iyer (VI092009A): Defer to shared initializeTIFStreamName() function whose code was cut & paste from here
% 5/24/10 Vijay Iyer (VI052410A): Use 'drawnow' instead of 'drawnow expose' here -- the goal in this case actually /is/ to flush callbacks!
% 7/14/10 Vijay Iyer (VI071410A): Use state.init.hAcqTasks instead of constructing local acqTaskList variable
% VI092210A: Check state.files.autosave instead of now defunct state.acq.saveDuringAcquisition -- Vijay Iyer 9/22/10
% VI092210B: Only check if tifStream object exists on first slice of stack (if applicable) -- Vijay Iyer 9/22/10
% VI092310A: Update the new lastStartMode flag variable -- Vijay Iyer 9/23/10
% VI100610A: Arm triggers and start exported clock Tasks, now separately from the 'acquisition' Tasks -- Vijay Iyer 10/6/10
% VI121710A: Add acquisitionStarted event -- Vijay Iyer 12/17/10
% VI010711A: Account for numberOfZSlices and averaging vars in determining the framesPerFile value for acquisition -- Vijay Iyer 1/7/11
%
%% CREDITS
%   Created 9/3/09, by Vijay Iyer
%   Based heavily on earlier version by Tom Pologruto, CSHL, February 2001
%
%% ***********************************************************************
global state

%VI030708A
if state.files.autoSave %VI092210A
    %     %standardModeGUI('cbSaveDuringAcq_Callback',gh.standardModeGUI.cbSaveDuringAcq,[],guidata(gh.standardModeGUI.cbSaveDuringAcq)); %VI063010A: Removed (finally) %Probably not necessary--but force through that logic anyway
    %     if state.acq.saveDuringAcquisition  %verifies that state hasn't changed
    if state.acq.averaging
        storedNumberOfFrames = state.acq.numberOfFrames / state.acq.numAvgFramesSave; % Guaranteed to be an even division, if state.acq.averaging=1
    else     
        storedNumberOfFrames = state.acq.numberOfFrames; 
    end
    
    if storedNumberOfFrames * state.acq.numberOfZSlices <= state.acq.framesPerFileGUI %VI010711A %VI082108B
        %state.acq.framesPerFile = state.acq.numberOfFrames;
        state.acq.framesPerFile = inf;
    else
        state.acq.framesPerFile = state.acq.framesPerFileGUI;
    end
    updateHeaderString('state.acq.framesPerFile'); %VI062308A
    
	if state.acq.framesPerFile &&  state.internal.zSliceCounter == 0
        if ~isempty(state.files.tifStream)
            fileName = get(state.files.tifStream, 'filename');
            if exist(fileName,'file')
                errordlg(['A TIF stream associated with the file ''' fileName ''' is still open. That file may be corrupt. The stream is now being forcibly closed to allow future GRABs']);
            else
                fprintf(2,'WARNING: A TIF stream was found already open. This is strange, but shouldn''t happen if trying the GRAB again.');
            end
            delete(state.files.tifStream,'leaveFile');
            state.files.tifStream = [];
            abortCurrent;
        else
            initializeTIFStreamName(); %VI092009A
        end
    end
else
    state.acq.framesPerFile = Inf; %VI062308A (this was set to 1; inf signals that all teh frames will be saved to a single file)
    updateHeaderString('state.acq.framesPerFile'); %VI062308A
end

%acqTaskList = [state.init.hAI state.init.hAO]; %VI071410A %VI091909A

if state.init.eom.pockelsOn == 1  
    %Write output data to Pockels AO Task
    state.init.eom.hAO.writeAnalogData(makePockelsCellDataOutput(state.init.eom.grabLaserList)); 

    %Configure output buffering based on whether there is, or isn't, power modulation
    if si_isPureContinuous()
        state.init.eom.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_ContSamps'); %Buffer length equals length of acquisition...no need to repeat
    elseif state.init.eom.usePowerArray || any(state.init.eom.showBoxArray) || any(state.init.eom.uncagingMapper.enabled)
        state.init.eom.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps', state.init.eom.hAO.get('bufOutputBufSize')); %Buffer length equals length of acquisition...no need to repeat
    else
        state.init.eom.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps', state.init.eom.hAO.get('bufOutputBufSize') * state.acq.numberOfFrames); %Buffer length is of a single frame
    end

    %%%VI091909A: Removed %%%%%%
    %     %Configure Pockels AO triggering
    %     setTriggerSource(state.init.eom.hAO,false); %VI041308A
    %
    %     %Start Pockels AO Task
    %     start(state.init.eom.hAO);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%

    %acqTaskList = [acqTaskList state.init.eom.hAO]; %VI071410A %VI091909A    
end

if si_isPureContinuous()
    state.init.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_ContSamps');
else    
    state.init.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps', state.init.hAO.get('bufOutputBufSize') * state.acq.numberOfFrames);
end

%TODO: Consider avoiding this call if Looping and past first Repeat, and not resuming paused Loop
state.init.hAI.everyNSamplesEventCallbacks = state.hSI.hMakeFrameByStripes;

%setTriggerSource([state.init.hAO state.init.hAI],false); %VI091909A
%start([state.init.hAI state.init.hAO]); %VI091909A
% 
% if ~isempty(state.init.hFrameClkCtr)
%     if si_isPureContinuous()
%         state.init.hFrameClkCtr.cfgImplicitTiming('DAQmx_Val_ContSamps');
%     else
%         state.init.hFrameClkCtr.cfgImplicitTiming('DAQmx_Val_FiniteSamps');
%     end
% end
% if ~isempty(state.init.hLineClkCtr)
%     if si_isPureContinuous()
%         state.init.hLineClkCtr.cfgImplicitTiming('DAQmx_Val_ContSamps');
%     else
%         state.init.hLineClkCtr.cfgImplicitTiming('DAQmx_Val_FiniteSamps');
%     end
% end

%Flush any errant callbacks
state.internal.stopActionFunctions = 1;
drawnow; %VI052410A
state.internal.stopActionFunctions = 0; %VI032010A


armTriggers(state.init.hAcqTasks, state.acq.hClockTasks, false, true, true); %VI100610A %VI071410A %VI091909A
exportClocks(state.acq.numberOfFrames);%TO091210B %This has to come after `armTriggers`, to potentially disable triggering for gated clocks.
start([state.init.hAcqTasks state.acq.hClockTasks]); %VI100610A %VI071410A %VI091909A

state.internal.lastStartMode = 'grab/loop'; %VI092310A

notify(state.hSI,'acquisitionStarted',state.internal.lastStartMode); %VI121710A

return;