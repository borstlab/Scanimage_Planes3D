function setImagesToWhole
%% function setImagesToWhole
%   Function that redraws acquisition windows based on acquired data. Used during resize,etc.
%
%% MODIFICATIONS
%   VI022308A Vijay Iyer 2/23/08 - Handle merge channel figure
%   VI111708A Vijay Iyer 11/17/08 - Handle case where blue merges as gray
%   VI111708B Vijay Iyer 11/17/08 - Remove unnecessary warning message
%   VI011109A Vijay Iyer 1/11/09 - Handle 4-color merge 
%   VI021109A Vijay Iyer 2/11/09 - Don't set merge figure EraseMode; this is deferred to selectNumberOfStripes()
%   VI092210A Vijay Iyer 9/22/10 - The state.acq.acquiredData var is now a frame-indexed, reverse-chronological running buffer; changes made also have effect of displaying the most-recent, rather than first, frame acquired (the better behavior)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 
global state

if isstruct(state) && isfield(state,'init') && isfield(state,'acq') 
    updateMerge=false;
    if isfield(state.acq,'channelMerge') && state.acq.channelMerge %VI022308A (copying above convention of checking for field existence--probably stupid?)
        if ~isempty(state.acq.acquiredData) && iscell(state.acq.acquiredData)
            for i=1:state.init.maximumNumberOfInputChannels %VI011109A
                if ~isempty(state.acq.acquiredData{1}{i}) %VI092210A        
                    acqSize = size(state.acq.acquiredData{1}{i}); %VI092210A
                    mergeData = uint8(zeros([acqSize(1) acqSize(2) 3]));
                    updateMerge=true;
                    break;
                end
            end
        end
    end

    for channelCounter = 1:state.init.maximumNumberOfInputChannels
        if isfield(state.acq,'acquiringChannel')
            if state.acq.acquiringChannel(channelCounter)                  
                if ~isempty(state.acq.acquiredData) && iscell(state.acq.acquiredData) && ~isempty(state.acq.acquiredData{1}{channelCounter}) %VI092210A
                    
                    if state.acq.averagingDisplay
                        if state.internal.stripeCounter == 0
                            startIdx = 1;
                        else %Was aborted in the middle of a frame
                            startIdx = 2; 
                        end             
                        
                        stopIdx = min(startIdx + state.acq.numAvgFramesDisplay,length(state.acq.acquiredData));
                        
                        avgFactor = stopIdx - startIdx + 1;
                                                
                        frameData = state.acq.acquiredData{startIdx}{channelCounter}/avgFactor;
                        
                        if startIdx == 2 && isempty(find(frameData,1)) % Case where acquisition was stopped during first frame
                            frameData = state.acq.acquiredData{startIdx}{channelCounter};
                        else                            
                            avgFrameCount = 1;
                            for i=(startIdx+1):stopIdx
                                if isempty(find(state.acq.acquiredData{i}{channelCounter},1)) %assume this and all subsequent frames in buffer are all-zero
                                    frameData = frameData * (avgFactor/avgFrameCount); %Average only the valid frames
                                    break;
                                else
                                    frameData = frameData + state.acq.acquiredData{i}{channelCounter}/avgFactor;
                                    avgFrameCount = avgFrameCount + 1;                                    
                                end
                            end
                        end
                        
                    else 
                        % if the acqusition ended before all stripes were acquired, use the second-newest data for the missing stipes.
                        % TODO: should 'state.internal.stripeCounter' be reset after this operation?
                        if state.internal.stripeCounter < state.internal.numberOfStripes
                            linesPerStripe = state.acq.linesPerFrame/state.internal.numberOfStripes;
                            cellIndex = 1;
                            for i = 1:state.internal.numberOfStripes
                                startLine = (i-1)*linesPerStripe + 1;
                                stopLine = i*linesPerStripe - (i == state.internal.numberOfStripes)*state.acq.slowDimDiscardFlybackLine;
                                frameData(startLine:stopLine,:) = state.acq.acquiredData{cellIndex}{channelCounter}(startLine:stopLine,:);
                                if i == state.internal.stripeCounter
                                    cellIndex = 2;
                                end
                            end
                        else
                            frameData = state.acq.acquiredData{1}{channelCounter};
                        end
                    end
                    
                    
                    set(state.internal.imagehandle(channelCounter),'CData', frameData(:,:),... %VI092210A
                        'YData',[1 size(frameData,1)]); %VI092210A
                    if updateMerge %VI022308A, VI011109A
                        %%%VI011109A%%%%%%%%%%%
                        if state.acq.mergeColor(channelCounter) <= 3
                            mergeData(:,:,state.acq.mergeColor(channelCounter)) = uint8(((double(frameData(:,:))-state.internal.lowPixelValue(channelCounter))/(state.internal.highPixelValue(channelCounter)-state.internal.lowPixelValue(channelCounter)) * 255)); %VI092210A                       
                        elseif state.acq.mergeColor(channelCounter) == 4
                            chanImage = uint8(((double(frameData(:,:))-state.internal.lowPixelValue(channelCounter))/(state.internal.highPixelValue(channelCounter)-state.internal.lowPixelValue(channelCounter)) * 255)); %VI092210A
                            mergeData = mergeData + repmat(chanImage,[1 1 3]);
                        end
                        %%%%%%%%%%%%%%%%%%%%%%%%
                    end
                end                   
            end
        end
    end
    
   
    if updateMerge %VI022308A
        %%%VI011109A: Removed %%%%%%%%%%
        %         %%%VI111708A%%%%%%%
        %         if state.acq.acquiringChannel(3) && state.acq.mergeBlueAsGray
        %             mergeData(:,:,1) = mergeData(:,:,1) + mergeData(:,:,3);
        %             mergeData(:,:,2) = mergeData(:,:,2) + mergeData(:,:,3);
        %         end
        %         %%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        set(state.internal.mergeimage,'CData',mergeData,'YData',[1 size(mergeData,1)]); %VI021109A
        
        state.acq.acquiredDataMerged = mergeData;
    end
end

