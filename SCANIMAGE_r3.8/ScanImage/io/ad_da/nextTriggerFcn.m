function nextTriggerFcn(~,~)
%% function nextTriggerFcn(src,evnt)
%   Callback invoked by Next Trigger
%
%% CHANGES
%   VI092210A: state.acq.saveDuringAcquisition is now defunct-- Vijay Iyer 9/22/10
%   VI092310A: Set new state.internal.freshData flag when this function marks first trigger of acquisition -- Vijay Iyer 9/23/10
%   VI100410A: Add new built-in EventManager event -- Vijay Iyer 10/4/10%   VI111110A: Pseudo-focus mode has been eliminated; no longer check for this -- Vijay Iyer 11/11/10
%   VI111110A: Pseudo-focus mode has been eliminated; no longer check for this -- Vijay Iyer 11/11/10
%
%% CREDITS
%   Created 9/19/09, by Vijay Iyer
%% ************************************************

global state gh

if si_isPureContinuous
    
    %%Critical section -- redundant between here and acquisitionStartedFcn() for max execution speed
    sampsSinceStart = state.init.hAI.get('readTotalSampPerChanAcquired');
    roughTime = clock();         
    sampsSinceStart = double(sampsSinceStart); %Required to perform math
    %%%%%    
    
    notify(state.hSI,'nextTriggerReceived'); %VI100410A
    
    %Handle first vs subsequent triggers
    if isempty(state.internal.triggerTimeFirst)  %First trigger
        %%%Debug Only%%%%      
        if ~state.acq.pureNextTriggerMode
           error('Expected to be in ''Next Trigger Only'' mode, but found otherwise. Logical error.');  
        end
        %%%%%%%%%%%%%%%%%%%        
       
        %Records start trigger time to header, after correcting for latency error
        recordStartTriggerTime(sampsSinceStart,roughTime);
        
        state.internal.freshData = true; %VI092310A
    else   
        %Record uncorrected trigger time, for next triggered files (have to live with latency error)
        state.internal.triggerTime = roughTime;  
        state.internal.triggerTimeString = clockToString(state.internal.triggerTime);
        updateHeaderString('state.internal.triggerTimeString');
        
        %Compute samples to assign to previous trigger
        state.internal.triggerFrameDelayMS = 1000 * (sampsSinceStart - (state.internal.totalFrameCounter * state.internal.samplesPerStripe * state.internal.numberOfStripes))/state.acq.inputRate;
        updateHeaderString('state.internal.triggerFrameDelayMS');
    end

else %VI100410A
    notify(state.hSI,'nextTriggerReceived'); %VI100410A
end
    

%Stop ongoing acquisitions 
if ~state.acq.nextTrigAutoAdvance || ~state.internal.looping %'Advance' mode only applies for LOOP acquisition
    if state.acq.nextTrigStopImmediate
        %DEQ20101222endAcquisition(true); %Call in abort mode
        feval(state.hSI.hEndAcquisition,true);
    else
        %TODO: Handle end-of-frame stop mode
    end
else %'Advance' acquisition
    
    %Handle gap vs no-gap cases
    if state.acq.nextTrigAdvanceGap %VI111110A %VI092210A 
        if state.acq.nextTrigStopImmediate
            %DEQ20101222endAcquisition(true); %Call in abort mode %This call increments state.files.fileCounter
            feval(state.hSI.hEndAcquisition,true);
            
            dioTrigger(); %Internally trigger next acquisition
        else
            %TODO: Handle end-of-frame stop mode
        end
    else %Gap-free
        if state.acq.nextTrigStopImmediate
            
            %Functionality normally in endAcquisition()
            closeTIFStream(); %This call increments state.files.fileCounter
            
            %Functionality normally in startGrab()
            initializeTIFStreamName(); 
%             state.internal.frameCounter = 1; %Resets to first frame
			state.internal.frameCounter = 0;
            set(gh.mainControls.framesDone,'String',num2str(state.internal.frameCounter)); %Update frameCounter display (reflects frame count, rather than # frames/done; use 'set' rather than updateGUIByGlobal() to speed performance
            
            %Functionality normally implemented as startTrigger event
            acquisitionStartedFcn([],[],false); %Flag that this is invoked indirectly
        else
            %TODO: Handle end-of-frame stop mode
        end        
    end       
    
end






end

