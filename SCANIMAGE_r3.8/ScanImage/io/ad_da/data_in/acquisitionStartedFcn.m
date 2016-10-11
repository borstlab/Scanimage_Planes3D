function acquisitionStartedFcn(~,~,trueStart)
%% function acquisitionStartedFcn(trueStart)
%ACQUISITIONTRIGGEREDFCN Callback function invoked upon triggering of GRAB/LOOP acquisition%
%% SYNTAX
%   trueStart: (OPTIONAL) Logical indicating that this is a 'true' start trigger
%
%% NOTES
%   Code here mostly copied/pasted out of original makeFrameByStripes() -- Vijay Iyer 7/1/09
%
%   The tasks in this functions are best not done in the EveryNSamples, because they can potentially be done before the first batch of samples comes available. Also helps to balance workload for SamplesAcquiredFcn calls.
%
%% CHANGES
%   VI090109A: Changes related to using new DAQmx interface -- Vijay Iyer 9/1/09
%   VI092009A: Disallow multiple calls from hStartTrigCtr -- Vijay Iyer 9/20/09
%   VI092209A: Improve accuracy of trigger time measurement by determining the number of samples acquired -- Vijay Iyer 9/22/09
%   VI092209B: Allow the two cases this function logic is called--upon a 'true' start or via a next trigger -- to be distinguished -- Vijay Iyer 9/22/09
%   VI102209A: Use state.internal.storedLinesPerFrame where appropriate -- Vijay Iyer 10/22/09
%   VI102609A: Improve error message for case where creation of TIF file of less than 2 strips is attempted (and fails). Abort acquisition after, not before, displaying error message, so that @scim_tifStream message at least appears correctly on command line. -- Vijay Iyer 10/26/09
%   VI102809B: Display loop iteration command line info here now, instead of in mainLoop() -- Vijay Iyer 10/28/09
%   VI102909A: Store state.internal.stackTriggerTime value on first slice within a stack; only display to command line on first slice; use state.internal.stackTriggerTime as lastTriggerTime  -- Vijay Iyer 10/29/09
%   VI110109A: Set status string 'Acquiring...' here, so that it gets displayed during Cycle mode operation. -- Vijay Iyer 11/01/09
%   VI110509A: Initialize state.internal.tempImage for case where averaging. -- Vijay Iyer 11/5/09
%   VI111609A: Don't open @tifstream object if snapping -- Vijay Iyer 11/16/09
%   VI113009A: Don't bother determining state.internal.stackTriggerTimeString or storing to header -- the triggerTime stored to header always reflects first slice in acquisition anyway. Information is redundant. -- Vijay Iyer 11/30/09
%   VI091910A: Prevent subsequent triggers during an acquisition from having an effect -- Vijay Iyer 9/19/10
%   VI092210A: Check state.files.autosave instead of now defunct state.acq.saveDuringAcquisition; moved tifStream initialization to section executed only on first slice -- Vijay Iyer 9/22/10
%   VI092310A: The runningData buffer has been eliminated -- use the acquireData buffer as template for initializing averaging data buffer -- Vijay Iyer 9/23/10
%   VI092310B: Set new state.internal.freshData flag -- Vijay Iyer 9/23/10
%   VI100410A: Add new built-in EventManager event -- Vijay Iyer 10/4/10
%   VI022311A: BUGFIX -- Only apply 'OverrideLz' if stack start/endpoints have been selected  -- Vijay Iyer 2/23/11
%
%% CREDITS
%   Created 7/1/09, by Vijay Iyer
%% ************************************************

global state

%Handle cases of stopped/aborted acq
if state.internal.stopActionFunctions || state.internal.abortActionFunctions
    return;
end

%tic;

%%%VI092209A: Removed%%%%%%
% %Record/store trigger time 
% %state.internal.triggerTime = get(obj,'InitialTriggerTime'); % Use the trigger time from DAQ Toolbox DAQ engine %VI090109A: Removed
% state.internal.triggerTime = clock(); %VI090109A
% state.internal.triggerTimeString = clockToString(state.internal.triggerTime); %VI090109A
% % if isempty(state.internal.triggerTime) || any(state.internal.triggerTime == 0)
% %     state.internal.triggerTimeString = '';
% %     fprintf(2, 'Warning: state.internal.triggerTime is not valid, hardware triggering may not have occurred.\n');
% % else
% %     state.internal.triggerTimeString = clockToString(state.internal.triggerTime); %clockToString is faster than datestr()
% % end
% updateHeaderString('state.internal.triggerTimeString');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI092209A%%%%%%%%%%%%%%%
%%Critical section -- redundant between here and nextTriggerFcn() for max execution speed
sampsSinceStart = state.init.hAI.get('readTotalSampPerChanAcquired');
roughTime = clock();
sampsSinceStart = double(sampsSinceStart);
%%%%%%%%%%%%%%%%%%%%%%%

notify(state.hSI,'startTriggerReceived'); %VI100410A

%%%VI091910A: Prevent subsequent triggers during an acquisition from having an effect
state.init.hStartTrigCtr.stop(); 

if nargin < 3 %VI092209B
    trueStart = true;
end

if trueStart %VI092209B
    %Records start trigger time to header, after correcting for latency error
    recordStartTriggerTime(sampsSinceStart,roughTime);
    
    state.internal.freshData = true; %VI092310B
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

%Stop trigger timer, if running 
if ~isempty(state.internal.triggerTimer) && strcmpi(get(state.internal.triggerTimer,'Running'),'on')
    stop(state.internal.triggerTimer);
end

%Record soft trigger time for case of external triggering
if state.acq.externallyTriggered
    %state.internal.softTriggerTime = clock; %VI090109A
    %state.internal.softTriggerTimeString = clockToString(state.internal.softTriggerTime); %VI090109A
    state.internal.softTriggerTime =state.internal.triggerTime; %VI090109A
    state.internal.softTriggerTimeString = state.internal.triggerTimeString; %VI090109A
    updateHeaderString('state.internal.softTriggerTimeString');
end

%VI102809B: Cache last trigger time
lastTriggerTime = state.internal.stackTriggerTime; %VI102909A
%VI102909A: Handle things applicable to first slice (or only slice) in stack
if state.internal.zSliceCounter == 0
    %%%VI102809B%%%%%%%%
    if state.internal.looping && ~isempty(state.internal.triggerTime) %Should arguably have ensured that state.internal.triggerTime is not empty above -- Vijay Iyer 10/28/09
        disp(['Starting ''' state.configName ''' at ' clockToString(state.internal.triggerTime)]);
        if ~isempty(lastTriggerTime)
            disp(['   Seconds since last acquisition: ' num2str(etime(state.internal.triggerTime,lastTriggerTime))]);
        end
    end
    %%%%%%%%%%%%%%%%%%%%%
    
    state.internal.stackTriggerTime = state.internal.triggerTime;
    %state.internal.stackTriggerTimeString = state.internal.triggerTimeString; %VI113009A
    
    %%%Initialize tifStream, if needed
    if state.files.autoSave && state.acq.framesPerFile && ~state.internal.snapping %VI092210A %VI111609A: Ensure not snapping %Checks to ensure not in 'pseudo-focus' mode
        try
            state.files.tifStream = scim_tifStream(state.files.tifStreamFileName,state.acq.pixelsPerLine, state.internal.storedLinesPerFrame, state.headerString); %VI102209A
        catch
            errordlg('Unable to initialize file for acquisition. Acquisition aborted.', 'Failed to Create File', 'modal'); %VI102609A
            disp(lasterr);
            abortCurrent; %VI102609A
            return;
        end
    end    
end
%%%%%%%%%%%%%%%%%%%%%

%VI010610A: Determine Lz array, if any, that is in effect for this acquisition
if state.init.eom.powerVsZActive && state.init.eom.powerVsZEnable
    if state.init.eom.powerLzOverride && ~isempty(state.init.eom.powerLzOverrideArray) %VI022311A
        state.init.eom.powerLzArray = state.init.eom.powerLzOverrideArray;
    else
        state.init.eom.powerLzArray = state.init.eom.powerLzStoredArray;
    end
end        

%VI110509A: Initialize state.internal.tempImage
if state.acq.averaging
    state.internal.tempImageSave = cell(1, state.init.maximumNumberOfInputChannels);
    %%%VI051410A%%%%%
    for i=1:state.init.maximumNumberOfInputChannels
        state.internal.tempImageSave{i} = zeros(size(state.acq.acquiredData{1}{i})); %VI092310A
    end
    %%%%%%%%%%%%%%%%%
end

%DEQ20110103: Initialize state.internal.tempImageDisplay
if state.acq.averagingDisplay
    state.internal.tempImageDisplay = cell(1, state.init.maximumNumberOfInputChannels);
    %%%VI051410A%%%%%
    for i=1:state.init.maximumNumberOfInputChannels
        state.internal.tempImageDisplay{i} = zeros(size(state.acq.acquiredData{1}{i})); %VI092310A
    end
    %%%%%%%%%%%%%%%%%
end

%startTime = toc();
%disp(['Took ' num2str(startTime*1000) ' ms to handle start of acquisition tasks']);

setStatusString('Acquiring...'); %VI110109A



end

