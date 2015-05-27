function preallocateMemory(verbose)
%% function preallocateMemory
%
% This function preallocates the appropriate memory for each acquisition mode.
%% SYNTAX
%   preallocateMemory()
%   preallocateMemory(verbose)
%       verbose: Logical value indicating, if true, to display warning info during function operation. If omitted, value assumed to be true.
%
%% NOTES
%   Function was rewritten on 9/22/10. To see previous version, look at MOLD versions. -- Vijay Iyer 9/22/10
%
%   The state.acq.acquiredData is now the one and only running data buffer
%   Buffer is organized as a cell array by frame, with each cell containing another cell array by channel
%   Unlike previous implementations, the state.acq.acquiredData buffer now runs in reverse sequential order -- i.e. the first element is the most recently collected frame
%   
%% CREDITS
% Created 9/22/10, by Vijay Iyer
%% ****************************************************************************

global state

if nargin < 1
    verbose = true;
end

%Determine total # of frames available to be buffered
totalNumFrames = state.acq.numberOfZSlices * state.acq.numberOfFrames;

%Determine max # of frames to buffer 
if state.files.autoSave 
    maxNumBufferedFrames = state.internal.maxNumFramesStreaming;
else
    maxNumBufferedFrames = state.internal.maxNumFramesBuffered;
end

%Determine # of frames to actually buffer for given #Frames/#Slices -- 
%ensuring integer # of slices is buffered
if state.acq.numberOfZSlices > 1
    %Ensure an integer # of slices, within maxNumBufferedFrames limit
    minAcquiredDataLength = min(floor(maxNumBufferedFrames/state.acq.numberOfFrames) * state.acq.numberOfFrames, totalNumFrames);
    
    %Ensure at least one slice is buffered (possibly exceeding maxNumBufferedFrames limit)
    if minAcquiredDataLength == 0 %Was unable to buffer 1 slice
        minAcquiredDataLength = state.acq.numberOfFrames;
    end
else
    minAcquiredDataLength = min(maxNumBufferedFrames,totalNumFrames); 
end

%Ensure acquiredDataLength is sufficient for averaging (and is at least 2 - needed for setImagesToWhole())
acquiredDataLength = max([minAcquiredDataLength state.acq.numAvgFramesDisplay+1 state.acq.numAvgFramesSave+1]);

%Prepare the acquiredData buffer
numRows = state.internal.storedLinesPerFrame; %Sometimes a line/frame may be skipped
numColumns = state.acq.pixelsPerLine;

state.acq.acquiredData = cell(acquiredDataLength,1);
for channelCounter = 1:state.init.maximumNumberOfInputChannels

    allocateChan = state.acq.(['acquiringChannel' num2str(channelCounter)]); %Acquiring data for channel
    for frame=1:acquiredDataLength
        if channelCounter==1
            state.acq.acquiredData{frame} = cell(state.init.maximumNumberOfInputChannels,1);
        end
        if allocateChan
            state.acq.acquiredData{frame}{channelCounter}=zeros(numRows,numColumns,'uint16');
        else
            state.acq.acquiredData{frame}{channelCounter} = [];
        end
    end
end

%Allocate the 'maxData' buffer
state.acq.maxData = cell(1,state.init.maximumNumberOfInputChannels);

if state.acq.numberOfChannelsMax > 0
    state.acq.maxData = cell(1,state.init.maximumNumberOfInputChannels);
    for channelCounter = 1:state.init.maximumNumberOfInputChannels
        if state.acq.(['maxImage' num2str(channelCounter)])
            state.acq.maxData{channelCounter} = zeros(numRows,numColumns,'uint16');
        end
    end
end

% Allocate the merged channel data buffer
state.acq.acquiredDataMerged = zeros(numRows,numColumns,3);

%Set flags/vars for subsequent operations
state.internal.freshData = false;    

if ~state.files.autoSave 
    state.internal.dataBufferingLoss = length(state.acq.acquiredData) < totalNumFrames;
else
    state.internal.dataBufferingLoss = false;
end


%TODO: Determine how best to handle insufficient memory errors (though they should be rare now)    
% catch ME %Handle case where memory is insufficient
%     
%     switch ME.identifier
%         case 'MATLAB:nomem' %Insufficient memory
%             if state.standardMode.standardModeOn 
%                 if verbose 
%                     errordlg({'Insufficient memory to buffer intended acquisition.';...
%                         'Number of slices and frames reset to 1 each.'; ...
%                         '';...
%                         'Consider activating ''Save During Acquisition'' feature.'},'Insufficient Memory','modal');
%                 end
% 
%             else
%                 ME.rethrow(); 
%             end
%         otherwise
%             ME.rethrow();
%     end            
% end

    
