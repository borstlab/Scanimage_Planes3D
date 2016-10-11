function makeFrameByStripes(~,evnt)
%% function makeFrameByStripes(ai, SamplesAcquired)
% This is the 'SamplesAcqiredFcn' for GRAB/LOOP mode operation
% Takes data from data acquisition engine and formats it into a proper intensity image.
% Function also handles averaging, tracking of # frames/slices, and disk-logging capabilities
%
%% NOTES
%   This version was rewritten from scratch. To see earlier version of this function, see makeFrameByStripes.mold -- Vijay Iyer 5/13/10
%
%% CHANGES
%   VI060610A: Use newly created moveStackStartMove() helper to start move to next position in stack slice. This uses updated, more general, version of motorStartMove() -- Vijay Iyer 6/6/10
%   VI073010A Vijay Iyer 7/30/10 - Determine whether current stripe is the last stripe once, and store in 'lastStripe' -- Vijay Iyer 7/30/10
%   VI073010B Vijay Iyer 7/30/10 - BUGFIX: Fix dimension mismatch errors, and unnecessary augmentation of preallocated arrays, related to flyback line discard -- Vijay Iyer 7/30/10
%   VI090710A Vijay Iyer 9/7/10 - Read data is now passed in via the event structure, so no readAnalogData() call is needed
%   VI092210A: Check state.files.autosave instead of now defunct state.acq.saveDuringAcquisition -- Vijay Iyer 9/22/10
%   VI092210B: Move writeData() logic into this function -- all saves are streaming saves -- Vijay Iyer 9/22/10
%   VI092210C: Eliminate runningData buffer; state.acq.acquiredData is now a frame-indexed, reverse-chronological running buffer in all cases -- Vijay Iyer 9/22/10
%   VI092310A: Cache streamToDisk flag; do appropriate write operation for averaged acquisitions -- Vijay Iyer 9/23/10
%   VI100410A: Add new built-in EventManager event ('frameDone') -- Vijay Iyer 10/4/10
%   VI102610A: Add new built-in EventManager event ('stripeDone') -- Vijay Iyer 10/26/10
%   VI102810A: BUGFIX - Handle DiscardFlybackLine option correctly for Channel Merge display -- Vijay Iyer 10/28/10
%   VI010711A: BUGFIX - Was using focusFrameCounter rather than frameCounter, and re-initializing merge image every frame -- Vijay Iyer 1/7/11
%   VI022311A: BUGFIX - Actually use the state.acq.savingChannelX flags to determine whether to write data per-channel -- Vijay Iyer 2/23/11
%   VI080911A: Prevent extra motor move from starting after final slice has been collected -- Vijay Iyer 8/9/11
%   
%% CREDITS
%  Created 5/13/10, by Vijay Iyer
%  Based heavily on earlier version by Tom Pologruto
%% ********************************************************
global state gh

streamToDisk = state.files.autoSave && state.acq.framesPerFile && ~state.internal.snapping;  %VI092310A
grabStop = false; 
acquiredDataLength = length(state.acq.acquiredData); %VI092210C
%Handle cases of stopped/aborted acq
if state.internal.stopActionFunctions
    return;
end
if state.internal.abortActionFunctions
    abortInActionFunction;
    return
end

%Reset counters, if needed
if state.internal.forceFirst
    state.internal.stripeCounter=0;
    state.internal.forceFirst=0;
end

%Open the shutter if it's time
if state.shutter.shutterOpen==0
    if all(state.shutter.shutterDelayVector==[state.internal.frameCounter state.internal.stripeCounter])
        openShutter;
    end
end

if state.internal.stripeCounter==0  

    %Handle displayed seconds counter, which behaves differently for external/internally triggered cases
    %state.internal.triggerTime=clock(); %NOTE: This was in previous version, but commented out here. It appears like a (mostly harmless) bug, as the variable is computed, and then actually stored to header, in the acquisitionStartedFcn(). This appears to be superfluous, and to corrupt the state variable. -- Vijay Iyer 5/14/10
    if state.internal.looping==1 && ~state.acq.externallyTriggered %count-down timer
        state.internal.secondsCounter=max(round(state.acq.repeatPeriod-etime(clock,state.internal.stackTriggerTime)),0); 
    else %count-up timer
        state.internal.secondsCounter=floor(etime(clock,state.internal.stackTriggerTime)); 
    end

    set(gh.mainControls.secondsCounter,'String',num2str(state.internal.secondsCounter));
    
    %%%VI092210C%%%%
    % Do circular permutation of state.acq.acquiredData - first cell always contains most-recent frame data
    state.acq.acquiredData = [state.acq.acquiredData(acquiredDataLength); state.acq.acquiredData(1:(acquiredDataLength-1))];
    %%%%%%%%%%%%%%%%%
end

try
    %Is this the last time through this callback?
	if state.internal.frameCounter == state.acq.numberOfFrames - 1 && state.internal.stripeCounter==state.internal.numberOfStripes-1
        %state.internal.stopActionFunctions = 1; %would this be needed/useful?
        closeShutter;
        grabStop=true;
    end

    if state.internal.abortActionFunctions
        abortInActionFunction;
        return
    end
    
    %Stop acquisition, start motor move, and park scanner, as needed/appropriate
    if grabStop
        stopGrab();

        if state.acq.numberOfZSlices > 1 && state.internal.zSliceCounter < (state.acq.numberOfZSlices - 1) %VI080911A
            motorStackStartMove(); %VI060610A
            
            if state.acq.stackParkBetweenSlices
                scim_parkLaser('soft');
            end
        end                
    end

    %%%Determine start/stop lines and columns for data to get
    linesPerStripe=state.acq.linesPerFrame/state.internal.numberOfStripes;
    startLine = 1 + state.internal.stripeCounter*linesPerStripe;
    stopLine = startLine+linesPerStripe-1;
    stopLineLoopDiscard = stopLine; %VI073010B - stopLine value to use in handling line-discard cases within loop over channels

    %Compute start/end columns
    [startColumnForFrameData endColumnForFrameData] = determineAcqColumns();
    
   %%%Get the data
% tic;
    %frameFinalData = uint16(getdata(state.init.aiUDD, state.internal.samplesPerFrame/state.internal.numberOfStripes)); %VI090309A %VI050509A %VI042208A 
    %[ns,frameFinalData] = state.init.hAI.readAnalogData(state.internal.samplesPerFrame/state.internal.numberOfStripes,'native'); %VI090710A: Removed %VI090309A
    %%%VI090710A%%%%%%%%
    frameFinalData = evnt.data;
    if isempty(frameFinalData)
        error('Error during readAnalogData(): \t%s\n',evnt.errorMessage);
    end
    %%%%%%%%%%%%%%%%%%%%
% getTime = toc;

  
%     if length(frameFinalData) < state.internal.samplesPerLine * state.acq.linesPerFrame %AS messing start
%         frameFinalData(state.internal.samplesPerLine * state.acq.linesPerFrame,:) = 0;  %AS added : to actually pad because the thing is 2d)
%         fprintf(2, 'Padding stripe data from %s to %s with NULL values to syn. Temporary solution!\n', ...
%                 num2str(length(frameFinalData) + 1), num2str(state.internal.samplesPerLine * state.acq.linesPerFrame)); 
%     end %% AS messing end


    %%%Handle case where acquired data wraps beyond line period 'boundary'
    if endColumnForFrameData > state.internal.samplesPerLine
        if state.internal.numberOfStripes == 1            
            sampleShift = endColumnForFrameData - state.internal.samplesPerLine;
            startColumnForFrameData = startColumnForFrameData - sampleShift;
            endColumnForFrameData = state.internal.samplesPerLine;

            frameFinalData(1:sampleShift,:) = 0;
            frameFinalData = [frameFinalData(sampleShift+1:end,:);  frameFinalData(1:sampleShift,:)]; %Efficient circular shift of data
        else %this should have been prevented at start of acquisition
            fprintf(2,'WARNING (%s): Acquisition delay is too high. Aborting acquisition. Reduce acquisition delay or turn off image striping\n',mfilename);
            abortCurrent();
            return;
        end
    end

    lastStripe = (state.internal.stripeCounter == (state.internal.numberOfStripes - 1)); %VI073010A

    %%%Discard last line if indicated %%%%%%
    discardLineAfterReshape = false;
    discardLastLine =  state.acq.slowDimDiscardFlybackLine && lastStripe; %VI102810A

    if discardLastLine  %Final line of acquired data should be discarded %VI102810A %VI073010A
        stopLineLoopDiscard = stopLine - 1; %VI073010B
        discardLineAfterReshape = ~mod(state.acq.linesPerFrame,2); %For even # of lines - discard line after reshape
        if ~discardLineAfterReshape %For odd # of lines - discard line now, before reshape
            stopLine = stopLine - 1;
            linesPerStripe = linesPerStripe - 1;
            frameFinalData(end-state.internal.samplesPerLine+1:end,:) = [];
        end
    end    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    %Determine value of 'averaging' flags
    averagingSave = state.acq.averaging && state.acq.numberOfFrames > 1;
    averagingDisplay = state.acq.averagingDisplay && state.acq.numberOfFrames > 1;

    %%%VI092210C: REMOVED %%%%%%%
    %     %Determine state.acq.acquiredData index (can be frame counter, slice counter, or both, depending on case)
    %     if state.internal.keepAllSlicesInMemory && ~state.files.autoSave %VI092210A
    %         if ~averaging %store each individual frame and slice
    %             acquiredDataIdx =(state.internal.frameCounter + state.internal.zSliceCounter*state.acq.numberOfFrames);
    %         else
    %             acquiredDataIdx = state.internal.zSliceCounter + 1;
    %         end
    %     elseif averaging || state.files.autoSave %VI092210A
    %         acquiredDataIdx = 1;
    %     else
    %         acquiredDataIdx = state.internal.frameCounter;
    %     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if state.internal.abortActionFunctions
        abortInActionFunction;
        return
    end

    %Preallocate/initialize
    inputChannelCounter = 0;

%tic;   

    for channelCounter = 1:state.init.maximumNumberOfInputChannels
        if state.acq.acquiringChannel(channelCounter) % if statemetnt only gets executed when there is a channel to acquire.
            inputChannelCounter = inputChannelCounter + 1;
            if state.acq.(['pmtOffsetAutoSubtractChannel' num2str(channelCounter)])
                offset = state.acq.(sprintf('pmtOffsetChannel%d',channelCounter)); % get PMT offset for channel
            else
                offset=0;
            end
            
            invert = (-1)^state.acq.(['inputVoltageInvert' num2str(channelCounter)]); %VI122309A               
         
            if state.acq.bidirectionalScan
                temp = reshape(frameFinalData(:,inputChannelCounter) - offset,2*state.internal.samplesPerLine,linesPerStripe/2); %VI120511A
                temp_top = temp((startColumnForFrameData):(endColumnForFrameData),:);
                temp_bottom = flipud(temp((startColumnForFrameData+state.internal.samplesPerLine):(endColumnForFrameData+state.internal.samplesPerLine),:));

                if discardLineAfterReshape %VI073010B
                    if state.internal.averageSamples
                        currenttempImage = invert * reshape(mean(reshape([temp_top; temp_bottom],state.acq.binFactor,[]),1), state.acq.pixelsPerLine, linesPerStripe)';
                    else
                        currenttempImage = invert * reshape(sum(reshape([temp_top; temp_bottom],state.acq.binFactor,[]),1), state.acq.pixelsPerLine, linesPerStripe)';
                    end
                else
                    %Bin data
                    if state.internal.averageSamples
                        state.acq.acquiredData{1}{channelCounter}(startLine:stopLine,:) = ... %VI092210C
                        invert * reshape(mean(reshape([temp_top; temp_bottom],state.acq.binFactor,[]),1), state.acq.pixelsPerLine, linesPerStripe)'; %Do this in one line -- considerably faster this way.
                    else
                        state.acq.acquiredData{1}{channelCounter}(startLine:stopLine,:) = ... %VI092210C
                            invert * reshape(sum(reshape([temp_top; temp_bottom],state.acq.binFactor,[]),1), state.acq.pixelsPerLine, linesPerStripe)'; %Do this in one line -- considerably faster this way.
                    end
                end
            else
                currenttempImage = reshape(frameFinalData(:,inputChannelCounter) - offset, state.internal.samplesPerLine, linesPerStripe); %VI120511A % Converts data into proper shape for frame

                if discardLineAfterReshape %VI073010B
                    if state.internal.averageSamples
                        currenttempImage = invert * reshape(mean(reshape(currenttempImage(startColumnForFrameData:endColumnForFrameData,:),state.acq.binFactor,[]),1), state.acq.pixelsPerLine, linesPerStripe)' - offset; %VI062609A %VI071509A %VI090609A
                    else
                        currenttempImage = invert * reshape(sum(reshape(currenttempImage(startColumnForFrameData:endColumnForFrameData,:),state.acq.binFactor,[]),1), state.acq.pixelsPerLine, linesPerStripe)' - offset; %VI062609A %VI071509A %VI090609A
                    end
                else
                    %Bin data
                    if state.internal.averageSamples
                        state.acq.acquiredData{1}{channelCounter}(startLine:stopLine,:) = ... %VI092210C
                            invert * reshape(mean(reshape(currenttempImage(startColumnForFrameData:endColumnForFrameData,:),state.acq.binFactor,[]),1), state.acq.pixelsPerLine, linesPerStripe)' - offset; %VI062609A %VI071509A %VI090609A
                    else
                        state.acq.acquiredData{1}{channelCounter}(startLine:stopLine,:) = ... %VI092210C
                            invert * reshape(sum(reshape(currenttempImage(startColumnForFrameData:endColumnForFrameData,:),state.acq.binFactor,[]),1), state.acq.pixelsPerLine, linesPerStripe)' - offset; %VI062609A %VI071509A %VI090609A
                    end
                end
            end


            %%%VI073010B%%%%%%%
            if discardLineAfterReshape
                currenttempImage(end,:) = [];
                
                state.acq.acquiredData{1}{channelCounter}(startLine:stopLineLoopDiscard,:) = currenttempImage; %VI092210C
            end               
            %%%%%%%%%%%%%%%%%%%
            
            %For averaging case, store rolling sum into double array
            if averagingSave                                                
				avgCounterSave = mod(state.internal.frameCounter,state.acq.numAvgFramesSave) + 1;
                if avgCounterSave == 1
                    state.internal.tempImageSave{channelCounter}(startLine:stopLineLoopDiscard,:) = double(state.acq.acquiredData{1}{channelCounter}(startLine:stopLineLoopDiscard,:));  %VI092210C
                else
                    state.internal.tempImageSave{channelCounter}(startLine:stopLineLoopDiscard,:) = ((avgCounterSave - 1) * state.internal.tempImageSave{channelCounter}(startLine:stopLineLoopDiscard,:) ...
                        + double(state.acq.acquiredData{1}{channelCounter}(startLine:stopLineLoopDiscard,:)))/avgCounterSave; %VI092210C
                end
            end
            
            if averagingDisplay
                avgFactor = min(state.acq.numAvgFramesDisplay,length(state.acq.acquiredData));
                indices = startLine:stopLineLoopDiscard; %stripe indices                               
                
                if (state.internal.frameCounter + 1) == 1
                    state.internal.tempImageDisplay{channelCounter}(indices,:) = double(state.acq.acquiredData{1}{channelCounter}(indices,:));
                elseif (state.internal.frameCounter + 1) <= avgFactor
                    state.internal.tempImageDisplay{channelCounter}(indices,:) = ...
                        (state.internal.frameCounter * state.internal.tempImageDisplay{channelCounter}(indices,:) + double(state.acq.acquiredData{1}{channelCounter}(indices,:))) / (state.internal.frameCounter + 1);
                else
                    state.internal.tempImageDisplay{channelCounter}(indices,:) = ...
                        state.internal.tempImageDisplay{channelCounter}(indices,:) + (double(state.acq.acquiredData{1}{channelCounter}(indices,:)) - double(state.acq.acquiredData{avgFactor+1}{channelCounter}(indices,:))) / avgFactor;
                end

            end
          
        end
    end
%computeTime = toc();      

    if discardLineAfterReshape
        stopLine = stopLine - 1;
    end
    
%tic; 
    %Draw data
    for channelCounter = 1:state.init.maximumNumberOfInputChannels
       if state.acq.imagingChannel(channelCounter)
           if averagingDisplay               
               set(state.internal.imagehandle(channelCounter), 'CData', ...
                   state.internal.tempImageDisplay{channelCounter}(startLine:stopLine,:), 'YData', [startLine stopLine]);
           elseif ~averagingDisplay
               set(state.internal.imagehandle(channelCounter), 'CData', ...
                   state.acq.acquiredData{1}{channelCounter}(startLine:stopLine,:), 'YData', [startLine stopLine]); %VI092210C
           end           
       end        
    end
%drawTime = toc;    

    
%tic;
    if state.acq.channelMerge && ~state.acq.mergeFocusOnly
        if averagingDisplay
            makeMergeStripe(state.internal.tempImageDisplay,[startLine stopLine],discardLastLine); %VI102810A %VI092210C
        else
            makeMergeStripe(state.acq.acquiredData{1},[startLine stopLine],discardLastLine); %VI102810A %VI092210C
        end
    end
%mergeTime = toc;

    %Update figures/GUI status
    setStatusString('Acquiring...');

    %Signal stripeAcquired event
    notify(state.hSI,'stripeAcquired'); %VI102610A
   
    %Increment stripeCounter 
    state.internal.stripeCounter = state.internal.stripeCounter + 1;      

    %Handle end of frame (and acquisition), if reached
    if lastStripe %VI073010A %finished a frame!
        state.internal.stripeCounter = 0;
        state.internal.totalFrameCounter = state.internal.totalFrameCounter+1;                                                    
% tic;

		if ~notify(state.hSI,'frameAcquired'); %VI100410A
% disp('q');
			return;
		end
%disp('c');	

        %Write Data
        if streamToDisk  %VI092310A %VI092210A            
            
            if averagingSave && avgCounterSave == state.acq.numAvgFramesSave

                %Check to see if it's time to start a new file (i.e. a new frame chunk)
				frameCount =  (state.internal.frameCounter + 1)/state.acq.numAvgFramesSave + (state.internal.zSliceCounter) * state.acq.numberOfFrames/state.acq.numAvgFramesSave;
				handleFileChunking(frameCount);

				for channelCounter = 1:state.init.maximumNumberOfInputChannels % Loop through all the channels
					if ~isempty(state.internal.tempImageSave{channelCounter}) %VI092210C
						appendFrame(state.files.tifStream,state.internal.tempImageSave{channelCounter}(1:state.internal.storedLinesPerFrame,:)); %VI092210C %VI111609A
					end
				end

				state.internal.storedFrameCounter = state.internal.storedFrameCounter + 1;

            elseif ~averagingSave               
                
                %Check to see if it's time to start a new file (i.e. a new frame chunk)
				frameCount =  (state.internal.frameCounter + 1) + (state.internal.zSliceCounter) * state.acq.numberOfFrames;
				handleFileChunking(frameCount);
                
                %Append this frame's data to the current stream
                for channelCounter = 1:state.init.maximumNumberOfInputChannels % Loop through all the channels
                    if ~isempty(state.acq.acquiredData{1}{channelCounter}) %VI092210C
                        appendFrame(state.files.tifStream,state.acq.acquiredData{1}{channelCounter}(1:state.internal.storedLinesPerFrame,:)); %VI092210C %VI111609A
                    end
				end
				
				state.internal.storedFrameCounter = state.internal.storedFrameCounter + 1;
            end
        end
% writeTime = toc;

        %%%VI092210C: REMOVED %%%%%
        %         %Store recently acquired frame to running data buffer, if in use
        %         if runningDataLength
        %             for chanCount = 1:state.init.maximumNumberOfInputChannels
        %                 state.acq.runningData{1}{chanCount}= state.acq.acquiredData{chanCount}(:,:,acquiredDataIdx);
        %             end
        %         end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        
%         if state.internal.frameCounter == state.acq.numberOfFrames %finished the specified # Frames                                                         
		if state.internal.frameCounter == state.acq.numberOfFrames - 1
            set(gh.mainControls.framesDone,'String',num2str(state.internal.frameCounter + 1));
            %DEQ20101222endAcquisition;% ResumeLoop, parkLaser, Close Shutter, appendData, reset counters,...
            feval(state.hSI.hEndAcquisition);
        else           
            state.internal.frameCounter = state.internal.frameCounter + 1;	% Increments the frameCounter to ensure proper image storage and display
            set(gh.mainControls.framesDone,'String',num2str(state.internal.frameCounter)); %Update frameCounter display (reflects frame count, rather than # frames/done; use 'set' rather than updateGUIByGlobal() to speed performance
            drawnow expose; %VI110409A
        end
    end
    
    
%Use for profiling    
% fprintf(1,'Total Time=%05.2f \t GetTime=%05.2f \t ComputeTime=%05.2f \t DrawTime=%05.2f \t MergeTime=%05.2f \t WriteTime=%05.2f \n',1000*toc(hTotal),1000*getTime,1000*computeTime, 1000*drawTime,1000*mergeTime, 1000*writeTime);
% toc(hTotal)
catch ME

    if state.internal.abortActionFunctions
        abortInActionFunction;
        return
    else
        setStatusString('Error!');
        fprintf(2,'ERROR in callback function (%s): \t%s\n',mfilename,ME.message);
        most.idioms.reportError(ME);
    end
end

return;
    
%% HELPER FUNCTIONS

%Handle 'file chunking' dictated by frames/file values
function handleFileChunking(frameCount)
global state 

if ~isinf(state.acq.framesPerFile)  && (state.acq.framesPerFile == 1 || mod(frameCount + 1,state.acq.framesPerFile)==1) 
    close(state.files.tifStream);
	fileChunkCounter = ceil((frameCount + 1)/state.acq.framesPerFile);
    fileName  = [state.files.fullFileName '_' num2str(fileChunkCounter,'%03d') '.tif'];
    state.files.tifStream = scim_tifStream(fileName,state.acq.pixelsPerLine, state.internal.storedLinesPerFrame, state.headerString); %VI102209A
end

return;

%Paints a stripe of color-merged data based on the imageData at
function makeMergeStripe(imageData,yData,discardLastLine) %VI092210C: Eliminate posn argument

global state

yMask = yData(1):yData(2);

% if ((state.internal.frameCounter == 1 || state.acq.slowDimDiscardFlybackLine) && state.internal.stripeCounter == 0) || discardLastLine %VI010711A %VI102810A
    state.internal.mergeStripe = uint8(zeros([length(yMask) size(imageData{find(state.acq.acquiringChannel,1)},2) 3])); 
% else 
%     state.internal.mergeStripe(:) = 0; 
% end

for i=1:state.init.maximumNumberOfInputChannels
    if state.acq.acquiringChannel(i)
        if state.acq.mergeColor(i) <= 4
            chanImage = uint8(((double(imageData{i}(yMask,:))-state.internal.lowPixelValue(i))/(state.internal.highPixelValue(i)-state.internal.lowPixelValue(i)) * 255)); %VI092210C
            if state.acq.mergeColor(i) <= 3
                state.internal.mergeStripe(:,:,state.acq.mergeColor(i)) =  state.internal.mergeStripe(:,:,state.acq.mergeColor(i)) + chanImage;
            elseif state.acq.mergeColor(i) == 4
                state.internal.mergeStripe(:,:,1) = state.internal.mergeStripe(:,:,1) + chanImage;
                state.internal.mergeStripe(:,:,2) = state.internal.mergeStripe(:,:,2) + chanImage;
                state.internal.mergeStripe(:,:,3) = state.internal.mergeStripe(:,:,3) + chanImage;
            end
        end
    end
end

set(state.internal.mergeimage,'CData',state.internal.mergeStripe,'YData',yData); 
state.acq.acquiredDataMerged(yMask,:,:) = state.internal.mergeStripe;

return;

function motorStackStartMove() %VI060610A

global state

%Update ScanImage position state variables and display to next position, in advance of move completion
%Interrupted moves can lead to X/Y position shifts. So we re-use the initially determined X/Y position in setting each new slice position -- Vijay Iyer 10/16/08
%However, when secondary motor controller is moved - do NOT re-use the initial X/Y positions. 
%In this way, only one motor controller (primary or secondary) is ever used for the step to next slice
newZPosn = [];
if ~state.motor.motorZEnable %Move primary motor only. Covers 2 cases: 1) no secondary motor 2)XYZ-Z with motorZEnable=false    
    state.motor.absZPosition = state.internal.initialMotorPosition(3) - state.acq.stackCenteredOffset + state.acq.zStepSize * (state.internal.zSliceCounter + 1);        
    if state.motor.dimensionsXYZZ
        newZPosn = state.motor.absZPosition;
    else
        state.motor.absXPosition = state.internal.initialMotorPosition(1);
        state.motor.absYPosition = state.internal.initialMotorPosition(2);
    end    
    
else %Move secondary motor
    if state.motor.dimensionsXYZZ         
        state.motor.absZZPosition = state.internal.initialMotorPosition(4) - state.acq.stackCenteredOffset + state.acq.zStepSize * (state.internal.zSliceCounter + 1);
        newZPosn = state.motor.absZZPosition;
    else %XY-Z case
        state.motor.absZPosition = state.internal.initialMotorPosition(3) - state.acq.stackCenteredOffset + state.acq.zStepSize * (state.internal.zSliceCounter + 1);
        newZPosn = state.motor.absZPosition;
    end    
end            
motorUpdatePositionDisplay();

try 
    if isempty(newZPosn)
        motorStartMove(); %Move primary motor only, in XYZ
    else
        motorStartMove(newZPosn); %Move primary or secondary motor only, in Z only
    end
catch ME
    %if moveSecZ
    if state.motor.motorZEnable && state.motor.dimensionsXYZZ
        state.motor.absZZPosition = state.motor.absZZPosition - state.acq.zStepSize; %restore to previous value if move failed to start
    else
        state.motor.absZPosition = state.motor.absZPosition - state.acq.zStepSize; %restore to previous value if move failed to start
    end
    motorUpdatePositionDisplay();
    ME.rethrow();
end

%Scale the power for each beam (if Power vs Z feature is enabled for that beam) -- vectorized operation
if state.init.eom.pockelsOn && state.init.eom.powerVsZEnable  
    if state.motor.motorZEnable && state.motor.dimensionsXYZZ
        state.init.eom.stackPowerScaling = exp(state.init.eom.powerVsZEnable .* (state.motor.absZZPosition - state.internal.initialMotorPosition(4))./state.init.eom.powerLzArray); %VI010610A
    else        
        state.init.eom.stackPowerScaling = exp(state.init.eom.powerVsZEnable .* (state.motor.absZPosition - state.internal.initialMotorPosition(3))./state.init.eom.powerLzArray); %VI010610A
    end
end	

return;

%%%VI092210B/VI092310A%%%
function writeFrame()

global state

%Check to see if it's time to start a new file (i.e. a new frame chunk)
if ~isinf(state.acq.framesPerFile) && mod(state.internal.frameCounter,state.acq.framesPerFile)==1 && state.internal.frameCounter > state.acq.framesPerFile
    close(state.files.tifStream);
    fileChunkCounter = ceil(state.internal.frameCounter/state.acq.framesPerFile);
    fileName  = [state.files.fullFileName '_' num2str(fileChunkCounter,'%03d') '.tif'];
    state.files.tifStream = scim_tifStream(fileName,state.acq.pixelsPerLine, state.internal.storedLinesPerFrame, state.headerString); %VI102209A
end

%Append this frame's data to the current stream
for channelCounter = 1:state.init.maximumNumberOfInputChannels % Loop through all the channels
    if state.acq.savingChannel(channelCounter) && ~isempty(state.acq.acquiredData{1}{channelCounter}) %VI022311A %VI092210C
        appendFrame(state.files.tifStream,state.acq.acquiredData{1}{channelCounter}(1:state.internal.storedLinesPerFrame,:)); %VI092210C %VI111609A
    end
end

return;
%%%%%%%%%%%%%%%%%%%%%
