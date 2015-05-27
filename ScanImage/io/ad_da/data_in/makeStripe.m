function makeStripe(~,evnt)
%% function makeStripe(aiF, SamplesAcquired)
% This is the 'SamplesAcqiredFcn' for FOCUS mode operation
% Takes data from data acquisition engine and formats it into a proper intensity image.
%
%% NOTES
%   This version was rewritten from scratch. To see earlier versions of this function, see makeStripe.mold -- Vijay Iyer 2/14/09
%
%% CHANGES
%   VI032409A: Add warning dialog for case where acq delay is too large -- Vijay Iyer 3/24/09
%   VI050509A: Increase getdata() speed by using uddobject -- Vijay Iyer 5/5/09
%   VI052109A: Combine get and convert steps into one line, as this was found to reduce the execution time (particularly convert time) -- Vijay Iyer 5/21/09
%   VI062609A: Do pixel binning operation in one line, significantly improving performance (>2X) -- Vijay Iyer 6/26/09
%   VI071509A: Remove premature transpose causing mis-display in sawtooth scan mode -- Vijay Iyer 7/15/09
%   VI083109A: Handle changes to use new DAQmx interface -- Vijay Iyer 8/31/09
%   VI090409A: Stop FOCUS at the correct number of frames (fix of off-by-1 error) -- Vijay Iyer 9/4/09
%   VI090409B: Use new stopActionFunctions field to prevent excess callbacks -- Vijay Iyer 9/4/09
%   VI090609A: Bin correctly (as in not at all) when state.acq.binFactor=1 -- Vijay Iyer 9/6/09
%   VI090609B: Replace all use of state.internal.samplesPerLineF with state.internal.samplesPerLine -- Vijay Iyer 9/6/09
%   VI091009A: Separate out 'plot' operation from 'compute' operation, so that turning Imaging off for Channels actually reduces processing time. -- Vijay Iyer 9/10/09
%   VI091309A: Remove drawnow() call. It's not necessary for figure update..only adds unnecessary time. -- Vijay Iyer 9/13/09
%   VI091509A: Do circular shift with colon notation -- this is much faster than circshift() itself. -- Vijay Iyer 9/15/09
%   VI102209A: Move start/end line determination outside of channel loop -- Vijay Iyer 10/22/09
%   VI102209B: Handle slow dimension flyback options, handling case of both odd and even # of lines -- Vijay Iyer 10/22/09
%   VI102409A: Remove warning dialog when acq delay is too high; simply make acq delay control red -- Vijay Iyer 10/24/09
%   VI122309A: Handle inverted channel data case here -- must do directly now, rather than relying on DAQ Tool - Vijay Iyer 12/23/09
%   VI122909A: BUGFIX - Implement VI122309A correctly, by enclosing -1 in parentheses -  Vijay Iyer 12/29/09
%   VI090710A: Read data is now passed in via the event structure, so no readAnalogData() call is needed -- Vijay Iyer 9/7/10
%   VI092210A: The state.acq.acquiredData variable is now a frame-indexed, reverse-chronological running buffer -- Vijay Iyer 9/22/10
%   VI092310A: FOCUS operations should refresh/update the running/circular buffer, just as GRAB acquisitions -- Vijay Iyer 9/23/10
%   VI100410A: Add new built-in EventManager event ('frameDone') -- Vijay Iyer 10/4/10
%   VI102010A: state.internal.lastTimeDelay corrected to renamed state.internal.lastRepeatPeriod; restores ability to Focus during Loop waits -- Vijay Iyer 10/20/10
%   VI102610A: Add new built-in EventManager event ('stripeDone') -- Vijay Iyer 10/26/10
%   VI102810A: Improve error messaging from CATCH block -- Vijay Iyer 10/28/10
%   VI102810B: BUGFIX - Handle DiscardFlybackLine option correctly for Channel Merge display -- Vijay Iyer 10/28/10
%
%% CREDITS
%  Created 2/14/09, by Vijay Iyer
%  Based heavily on earlier version by Tom Pologruto
%% ********************************************************

global state gh
%hTotal = tic();

if state.internal.abortActionFunctions  || state.internal.stopActionFunctions %VI090409B
    return
end

if state.internal.forceFirst
    state.internal.stripeCounter=0;
    state.internal.forceFirst=0;
end

try
    %[computeTime, drawTime] = deal(0);
    wrapWarning = false;
    
    %Start of frame actions
    if state.internal.stripeCounter==0
        
        %Update timer
        if state.internal.looping==1
            if state.cycle.cycling
                countdownPeriod = state.cycle.iterationDelay;
            else
                countdownPeriod = state.acq.repeatPeriod;
            end
            state.internal.secondsCounter=floor(countdownPeriod-etime(clock,state.internal.triggerTime));
            updateGUIByGlobal('state.internal.secondsCounter');
        end
        
        %First frame actions
        if state.internal.focusFrameCounter == 0
            
            %Initialize averaging buffer
            if state.acq.averagingDisplay
                state.internal.tempImageDisplay = cell(1, state.init.maximumNumberOfInputChannels);
                for i=1:state.init.maximumNumberOfInputChannels
                    state.internal.tempImageDisplay{i} = zeros(size(state.acq.acquiredData{1}{i})); %VI092310A
                end
            end
        end
        
        %%%VI092310A%%%%%%
        % Do circular permutation of state.acq.acquiredData - first cell always contains most-recent frame data
        acquiredDataLength = length(state.acq.acquiredData);
        state.acq.acquiredData = [state.acq.acquiredData(acquiredDataLength); state.acq.acquiredData(1:(acquiredDataLength-1))];
        %%%%%%%%%%%%%%%%%%
        
        
    end
    
    %Compute start/end columns
    [startColumnForStripeData endColumnForStripeData] = determineAcqColumns();
    
    %%%VI102209A: Determine lines to get%%%%
    linesPerStripe=state.acq.linesPerFrame/state.internal.numberOfStripes;
    ydata=[(1 + (linesPerStripe*state.internal.stripeCounter)) (linesPerStripe*(1 + state.internal.stripeCounter))];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %tic;
    %stripeFinalData = uint16(getdata(state.init.aiFUDD, state.internal.samplesPerStripe)); % VI083109A %VI052109A %VI050509A % Gets enough data for one stripe from the DAQ engine for all channels present
    %stripeFinalData = uint16(stripeFinalData); %VI052109A
    %[ns,stripeFinalData] = state.init.hAIF.readAnalogData(state.internal.samplesPerStripe, state.internal.samplesPerStripe, 'native',1);
    %[ns,stripeFinalData] = state.init.hAI.readAnalogData(state.internal.samplesPerStripe,'native'); %VI090710A
    %%%VI090710A%%%%%%%%
    stripeFinalData = evnt.data;
    if isempty(stripeFinalData)
        error('Error during readAnalogData(): \t%s\n',evnt.errorMessage);
    end
    %%%%%%%%%%%%%%%%%%%%
    %getTime = toc;
    
    
    %tic;
    if endColumnForStripeData > state.internal.samplesPerLine %VI090609B
        if state.internal.numberOfStripes == 1  %Acquiring in frame-sized chunks
            sampleShift = endColumnForStripeData - state.internal.samplesPerLine; %VI090609B
            startColumnForStripeData = startColumnForStripeData - sampleShift;
            endColumnForStripeData = state.internal.samplesPerLine; %VI090609B
            
            stripeFinalData(1:sampleShift,:) = 0;
            stripeFinalData = [stripeFinalData(sampleShift+1:end,:);  stripeFinalData(1:sampleShift,:)]; %VI091509A
            %stripeFinalData = circshift(stripeFinalData,-sampleShift); %VI091509A: Removed %final line of frame will contain extra 0s
        else
            wrapWarning = true;
            setStatusString('Acq Delay Too High!');
            set(gh.configurationControls.etAcqDelay,'BackgroundColor',[1 0 0]); %VI102409A
            %%%VI102409A: Removed %%%%%%%%%
            %             if isempty(state.internal.acqDelayWarnFig)
            %                 state.internal.acqDelayWarnFig = warndlg('Acquisition delay is too high. Either reduce acquisition delay or disable image striping.','Acq Delay Too High');
            %                 set(state.internal.acqDelayWarnFig,'DeleteFcn',@acqDelayWarnFigDeleteFcn);
            %             end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    else %VI102409A
        setStatusString('Focusing...'); %VI102409A: Moved from below
        set(gh.configurationControls.etAcqDelay,'BackgroundColor',[1 1 1]); %VI102409A
    end
    %wrapFixTime = toc;
    
    if length(stripeFinalData) < state.internal.samplesPerLine * state.acq.linesPerFrame / state.internal.numberOfStripes %VI090609B
        fprintf(2, 'WARNING: Data acquisition underrun. Expected to acquire %s samples, only found %s samples in the buffer.', ...
            num2str(state.internal.samplesPerLine * state.acq.linesPerFrame / state.internal.numberOfStripes), ... %VI090609B
            num2str(length(stripeFinalData)));
        if state.internal.compensateForBufferUnderruns
            stripeFinalData(state.internal.samplesPerLine * state.acq.linesPerFrame / state.internal.numberOfStripes) = 0; %VI090609B
            fprintf(2, 'Padding stripe data from %s to %s with NULL values. Image should be considered corrupted.\n         To disable this behavior, set state.internal.compensateForBufferUnderruns equal to 0.\n', ...
                num2str(length(stripeFinalData) + 1), num2str(state.internal.samplesPerLine * state.acq.linesPerFrame / state.internal.numberOfStripes)); %VI090609B
        end
    end
    
    %%%VI102209A: Discard last line if indicated %%%%%%
    discardLineAfterReshape = false;
    discardLastLine =  state.acq.slowDimDiscardFlybackLine && (state.internal.stripeCounter + 1) == state.internal.numberOfStripes; %VI102810B
    
    if discardLastLine %VI102810B
        ydata(2) = ydata(2)-1;
        discardLineAfterReshape = ~mod(state.acq.linesPerFrame,2); %For even # of lines - discard line after reshape
        if ~discardLineAfterReshape %For odd # of lines - discard line now, before reshape
            linesPerStripe = linesPerStripe - 1;
            stripeFinalData(end-state.internal.samplesPerLine+1:end,:) = [];
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    inputChannelCounter = 0;
    
    tempStripe = cell(state.init.maximumNumberOfInputChannels,1);
    
    %tic;
    for channelCounter = 1:state.init.maximumNumberOfInputChannels
        if state.internal.abortActionFunctions
            abortFocus;
            return;
        end
        
        if state.acq.acquiringChannel(channelCounter)  % if statement only gets executed when there is a channel to focus.
            if state.acq.(['pmtOffsetAutoSubtractChannel' num2str(channelCounter)])
                offset = state.acq.(sprintf('pmtOffsetChannel%d',channelCounter)); % get PMT offset for channel
            else
                offset=0;
            end
            
            invert = (-1)^state.acq.(['inputVoltageInvert' num2str(channelCounter)]); %VI122309A
            
            %%%VI102209A: Relocated%%%%%%%
            %linesPerStripe=state.acq.linesPerFrame/state.internal.numberOfStripes;
            %ydata=[(1 + (linesPerStripe*state.internal.stripeCounter)) (linesPerStripe*(1 + state.internal.stripeCounter))];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            inputChannelCounter = inputChannelCounter + 1;
            
            if wrapWarning
                tempStripe{channelCounter} = uint16(zeros(linesPerStripe,state.acq.pixelsPerLine));
            elseif state.acq.bidirectionalScan
                temp = reshape(stripeFinalData(:,inputChannelCounter) - offset,2*state.internal.samplesPerLine,linesPerStripe/2); %VI120511A %VI090609B
                
                temp_top = temp((startColumnForStripeData):(endColumnForStripeData),:);
                temp_bottom = flipud(temp((startColumnForStripeData+state.internal.samplesPerLine):(endColumnForStripeData+state.internal.samplesPerLine),:)); %VI090609B
                %tempStripe{channelCounter} = reshape([temp_top; temp_bottom],state.acq.samplesAcquiredPerLine,lps)'; %VI062609A
                
                %tempStripe{channelCounter} = add2d(tempStripe{channelCounter},state.acq.binFactor)-offset; %VI062609A
                
                if state.internal.averageSamples
                    tempStripe{channelCounter} = invert * reshape(mean(reshape([temp_top; temp_bottom],state.acq.binFactor,[]),1),state.acq.pixelsPerLine, linesPerStripe)'; %VI062609A %VI090609A %VI122309A
                else
                    tempStripe{channelCounter} = invert * reshape(sum(reshape([temp_top; temp_bottom],state.acq.binFactor,[]),1),state.acq.pixelsPerLine, linesPerStripe)'; %VI062609A %VI090609A %VI122309A
                end
            else
                tempStripe{channelCounter} = reshape(stripeFinalData(:, inputChannelCounter) - offset, ... %VI120511A
                    state.internal.samplesPerLine,linesPerStripe); %Extracts only Channel 1 Data %VI071509A: Don't transpose yet %VI090609B
                
                %Bin samples into pixels...
                %tempStripe{channelCounter} = add2d(tempStripe{channelCounter}(:, startColumnForStripeData:endColumnForStripeData), state.acq.binFactor)-offset; %VI062609A %add2d converts tempStripe to double format
                
                if state.internal.averageSamples
                    tempStripe{channelCounter} = invert * reshape(mean(reshape(tempStripe{channelCounter}(startColumnForStripeData:endColumnForStripeData,:),state.acq.binFactor,[]),1),state.acq.pixelsPerLine, linesPerStripe)'; %VI062609A, VI071509A, VI090609A, VI122309A
                else
                    tempStripe{channelCounter} = invert * reshape(sum(reshape(tempStripe{channelCounter}(startColumnForStripeData:endColumnForStripeData,:),state.acq.binFactor,[]),1),state.acq.pixelsPerLine, linesPerStripe)'; %VI062609A, VI071509A, VI090609A, VI122309A
                end
            end
            
            
            %%%VI09109A: Relocated below %%%%%%%%
            %             % Displays the current images on the screen as they are acquired.
            %             set(state.internal.imagehandle(channelCounter), 'CData', tempStripe{channelCounter}, ...
            %                 'YData',ydata);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%VI102209B: Discard last line if indicated %%%%%%
            if discardLineAfterReshape
                tempStripe{channelCounter}(end,:) = [];
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            state.acq.acquiredData{1}{channelCounter}(ydata(1):ydata(2),:)=tempStripe{channelCounter}; %VI092210A
            
            %%%VI080911A%%%
            if state.acq.averagingDisplay
                avgFactor = min(state.acq.numAvgFramesDisplay,length(state.acq.acquiredData));
                
                if (state.internal.focusFrameCounter + 1) == 1
                    state.internal.tempImageDisplay{channelCounter}(ydata(1):ydata(2),:) = double(tempStripe{channelCounter});
                elseif (state.internal.focusFrameCounter + 1) <= avgFactor
                    state.internal.tempImageDisplay{channelCounter}(ydata(1):ydata(2),:) = ...
                        (state.internal.focusFrameCounter * state.internal.tempImageDisplay{channelCounter}(ydata(1):ydata(2),:) + double(tempStripe{channelCounter})) / (state.internal.focusFrameCounter + 1);
                else
                    state.internal.tempImageDisplay{channelCounter}(ydata(1):ydata(2),:) = ...
                        state.internal.tempImageDisplay{channelCounter}(ydata(1):ydata(2),:) + (double(tempStripe{channelCounter}) - double(state.acq.acquiredData{avgFactor+1}{channelCounter}(ydata(1):ydata(2),:))) / avgFactor;
                end
            end
        end
    end
    %%%%%%%%%%%%%%%%%%
    
    %computeTime = toc();
    
    %%%VI091009A%%%%%%%%%%%%%%%%%%
    %tic;
    for channelCounter = 1:state.init.maximumNumberOfInputChannels
        if state.acq.imagingChannel(channelCounter)
            if state.acq.averagingDisplay
                set(state.internal.imagehandle(channelCounter), 'CData', state.internal.tempImageDisplay{channelCounter}(ydata(1):ydata(2),:), ...
                    'YData',ydata);
            else
                set(state.internal.imagehandle(channelCounter), 'CData', tempStripe{channelCounter}, ...
                    'YData',ydata);
            end
        end
    end
    %drawTime = toc;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Merge window update
    %tic;
    if state.acq.channelMerge
        %         if ((state.internal.focusFrameCounter == 1 || state.acq.slowDimDiscardFlybackLine) && state.internal.stripeCounter == 0) || discardLastLine %VI102810B %VI011109B
        state.internal.mergeStripe = uint8(zeros([size(tempStripe{find(state.acq.acquiringChannel,1)}) 3])); %VI111108A
        %         else
        %             state.internal.mergeStripe(:) = 0;
        %         end
        
        for i=1:state.init.maximumNumberOfInputChannels
            if state.acq.acquiringChannel(i)
                
                if state.acq.mergeColor(i) <=4 %Ensure that color is specified for this channel
                    if state.acq.averagingDisplay
                        chanImage = uint8((state.internal.tempImageDisplay{i}(ydata(1):ydata(2),:)-state.internal.lowPixelValue(i))/(state.internal.highPixelValue(i)-state.internal.lowPixelValue(i)) * 255);
                    else
                        chanImage = uint8((tempStripe{i}-state.internal.lowPixelValue(i))/(state.internal.highPixelValue(i)-state.internal.lowPixelValue(i)) * 255);
                    end
                    if state.acq.mergeColor(i) <= 3 %Red, green, or blue
                        state.internal.mergeStripe(:,:,state.acq.mergeColor(i)) = state.internal.mergeStripe(:,:,state.acq.mergeColor(i)) + chanImage;
                    elseif state.acq.mergeColor(i) == 4 %Grey
                        
                        state.internal.mergeStripe(:,:,1) = state.internal.mergeStripe(:,:,1) + chanImage;
                        state.internal.mergeStripe(:,:,2) = state.internal.mergeStripe(:,:,2) + chanImage;
                        state.internal.mergeStripe(:,:,3) = state.internal.mergeStripe(:,:,3) + chanImage;
                    end
                end
            end
        end
        
        set(state.internal.mergeimage,'CData',state.internal.mergeStripe,'YData',ydata);
        state.acq.acquiredDataMerged(ydata(1):ydata(2),:,:) = state.internal.mergeStripe;
    end
    %mergeTime = toc;
    
    %tic;
    %drawnow; %VI091309A
    %displayTime = toc;
    
    if ~wrapWarning
        setStatusString('Focusing...');
    end
    
    if state.internal.abortActionFunctions
        state.internal.stripeCounter=0;
        abortFocus;
        return;
    else
        notify(state.hSI,'stripeAcquired'); %VI102610A
        state.internal.stripeCounter = state.internal.stripeCounter + 1; % increments the stripecounter to ensure proper image displays
    end
    
    if  state.internal.stripeCounter == state.internal.numberOfStripes
        notify(state.hSI,'frameAcquired'); %VI100410A
        
        state.internal.stripeCounter = 0;
        state.internal.focusFrameCounter = state.internal.focusFrameCounter + 1;
    end
    
    if state.internal.focusFrameCounter == (state.internal.numberOfFocusFrames + 1) && ~state.acq.infiniteFocus  %VI090409A: Fix off-by-1 error
        state.internal.stripeCounter=0;
        state.internal.stopActionFunctions=1; %VI090409B
        endFocus;
    end
    if state.internal.abortActionFunctions
        state.internal.stripeCounter=0;
        abortFocus;
        return;
    end
    
    %Use for profiling
    %fprintf(1,'Total Time=%05.2f \t GetTime=%05.2f \t WrapFixTime=%05.2f \t ComputeTime=%05.2f \t DrawTime=%05.2f \t MergeTime=%05.2f \n',1000*toc(hTotal),1000*getTime,1000*wrapFixTime,1000*computeTime,1000*drawTime,1000*mergeTime);
catch ME
    if state.internal.abortActionFunctions
        return
    else
        setStatusString('Error!');
        fprintf(2,'ERROR in callback function (%s): \t%s\n',mfilename,ME.message);
        most.idioms.reportError(ME);
    end
end


%VI032409A: Add as subfunction, rather than nested function, to avoid any performance issues
function acqDelayWarnFigDeleteFcn(hObject,eventdata)
global state;
state.internal.acqDelayWarnFig = [];
return;
