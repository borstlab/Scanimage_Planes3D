function saveLastAcquisitionAs(numFrames)
%% function saveLastAcquisitionAs(numFrames)
% Saves the current data in memory to disk.
%
%% SYNTAX
%   numFrames: <OPTIONAL - Positive Integer> Specifies number of frames from state.acq.acquiredData running buffer to save to file. Value of Inf implies to save all buffered frames.
%              If omitted/empty: 1 frame is saved if last acquisition was FOCUS or if autoSave=true; otherwise, all buffered frames are saved
%
%
%% NOTES
% Can be accessed by selecting CTRL+S from Main Controls Window.
%
% At this time, function is only called with numFrames unsupplied (i.e. use default value computed herein) -- in future, there could be multiple Save As... functions/capabilities which employ the numFrames argument
%
% Note function must generally be called soon after an acquisition before any settings, including autoSave are changed
% Many relevant setting changes will cause the acquiredData buffer to be cleared (via preallocateMemory() call)
%
% For case of averaging and aborted acquisition, the most recently acquired frames, beyond an integer multiple of the number of averaged frames, are NOT saved
%
% For case of averaging, with autoSave off, averaging operation is presently done in this function
% This operation is redundant to that done during acquisition for display (and logging where enabled)
% This is awkward, but prevents need to maintain a separate averaged-frames buffer from the acquiredData running buffer (which stores all frames up to limit, regardless of averaging setting)
% This may be revisited in future
%
%% CHANGES
% TPMOD_1: Modified 12/31/03 Tom Pologruto - Fixed the status string updates.
% VI071608A Vijay Iyer 7/16/08 -- Fix the '.tif.tif' problem, which appears in newer Matlab versions because behavior of uiputfile has changed
% VI091508A Vijay Iyer 9/15/08 -- Ensure that files are saved in the selected directory, not the state.files.savePath directory
% VI091508B Vijay Iyer 9/15/08 -- Display name of saved file correctly
% VI092310A Vijay Iyer 9/23/10 -- Save the specified number of frames from the newly organized (frame-indexed, reverse-chronological) acquiredData running buffer. Use sensible default if no value supplied.
% VI102010A Vijay Iyer 10/20/10 -- Use appropriate default start path when specifying file; no longer change directory
% VI110210A Vijay Iyer 11/2/10 -- Use windows home-drive, rather than Matlab current directory, as the default directory
% VI111110A Vijay Iyer 11/11/10 -- Defer to most.idioms.startPath() logic to implement VI110210A
%
%% ************************************************************

global state gh

%Ensure there is (likely) data available to save
acquiredData = [];
if state.internal.freshData
    chanIdx = find(state.acq.acquiringChannel,1);
    if ~isempty(chanIdx)
        acquiredData = state.acq.acquiredData;
        
        %Extract only those frames that contain data
        for i=1:length(acquiredData)
            if isempty(find(acquiredData{i}{chanIdx}))
                acquiredData(i:end) = [];
                break;
            end
        end
    end
else
    % no acquired data--use CData, if it is available...
    for channelCounter = 1:state.init.maximumNumberOfInputChannels
        if state.acq.acquiringChannel(channelCounter)
            cdata = get(state.internal.imagehandle(channelCounter),'CData');
            if max(max(cdata)) > 0
                acquiredData{1}{channelCounter} = zeros(size(cdata));
                acquiredData{1}{channelCounter} = get(state.internal.imagehandle(channelCounter),'CData');
            end
        end
    end
end

if isempty(acquiredData)
    msgbox('No buffered data is available to be saved.','No Available Data','warn');
    return;
end

% Make the file name with the tif extension
stat=state.internal.statusString;
setStatusString('Saving Last Acq As...');
if isdir(state.files.savePath)
    startPath = state.files.savePath; %VI102010A
    %cd(state.files.savePath); %VI102010A: Removed
else
    startPath = state.files.rootSavePath; %VI102010A
    
    %%%VI110210A
    if isempty(startPath)
        startPath = most.idioms.startPath(); %VI111110A
    end
end
[fname, pname]=uiputfile('*.tif', 'Choose File name...',startPath); %VI102010A

if isnumeric(fname)
    setStatusString(stat);
    return
else
    %%%VI071608A -- only append '.tif' if needed
    [~,~,ext] = fileparts(fname);
    if isempty(ext)
        fileName=[pname fname '.tif'];
    else
        fileName = fullfile(pname,fname); %VI091508A
    end
    %%%%%%%%%
end

%%%VI092310A%%%%%%%%%%%%%%%%%%%%
if nargin < 1 || isempty(numFrames)
    %Use default numFrames value
    if strcmpi(state.internal.lastStartMode,'grab/loop')
        if state.files.autoSave
            numFrames = 1;
        else
            numFrames = inf;
        end
    else
        numFrames = 1;
    end
else
    validateattributes(numFrames,'numeric',{'integer' '>=' 1},1);
end


%Write data to disk, frame-wise
first = true;
writeOptions = {'Compression', 'none', 'Description', state.headerString};


if ~state.acq.averaging || numFrames == 1 ...
        || strcmpi(state.internal.lastStartMode,'focus') || strcmpi(state.internal.lastStartMode,'snap')
    
    numFrames = min(length(acquiredData),numFrames);
    
    for frameCounter=numFrames:-1:1 %Handle reverse chronological order
        writeFrame(acquiredData{frameCounter});
    end
    
else     %We must average data here

    %VI121110A: numFrames=inf in all actual use cases now
    numFramesToWrite = floor(min(length(acquiredData),numFrames) / state.acq.numAvgFramesSave); %Number of frames available to write (full average set)
    
    totalFrameCounter = length(acquiredData); %Start from end of valid acquiredData, i
    
    for i = 1:numFramesToWrite
        averageFrameBuffer = cell(state.init.maximumNumberOfInputChannels,1);
        frameClass = '';
        
        for frameCounter = 1:state.acq.numAvgFramesSave
            for channelCounter = 1:state.init.maximumNumberOfInputChannels
                
                currentMonoFrame = acquiredData{totalFrameCounter}{channelCounter};
                if ~isempty(currentMonoFrame)
                    if isempty(frameClass)
                        frameClass = class(currentMonoFrame);
                    end
                    
                    if isempty(averageFrameBuffer{channelCounter})
                        averageFrameBuffer{channelCounter} = double(currentMonoFrame);
                    else
                        averageFrameBuffer{channelCounter} = double(currentMonoFrame) + averageFrameBuffer{channelCounter};
                    end
                end
            end
            
            totalFrameCounter = totalFrameCounter - 1;
        end
        
        %Do averaging operation, on each channel
        averageFrameBuffer = cellfun(@(frameData)feval(frameClass,frameData/state.acq.numberOfFrames),averageFrameBuffer,'UniformOutput',false);
        
        %Save slice (with averaging on, data is saved by slice)
        writeFrame(averageFrameBuffer);
        
    end
end



    function writeFrame(frameData)
        for chanCount = 1:state.init.maximumNumberOfInputChannels
            if chanCount > length(frameData)
                break;
            end
            currentMonoFrame = frameData{chanCount};
            if ~isempty(currentMonoFrame)
                if first
                    imwrite(currentMonoFrame, fileName,'WriteMode', 'overwrite', writeOptions{:});
                    first = false;
                else
                    imwrite(currentMonoFrame, fileName,'WriteMode', 'append', writeOptions{:});
                end
            end
        end
    end

%Notify user if no frames are found
if numFrames==0 || first
    msgbox('No buffered data was available to be saved. File not saved.','No Available Data','warn');
else
    disp(['File ' fileName ' saved.']);
    setStatusString('File Saved');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end

%%%VI092310A: Removed %%%%%%%%%%%%
% for channelCounter = 1:state.init.maximumNumberOfInputChannels % Loop through all the channels
%     if getfield(state.acq, ['acquiringChannel' num2str(channelCounter)]) % If acquiring..
%         numberOfFrames = size(state.acq.acquiredData{channelCounter},3);
%     end
% end
% first=1;
% for frameCounter=1:numberOfFrames % Loop through all the frames
%     for channelCounter = 1:state.init.maximumNumberOfInputChannels % Loop through all the channels
%         if getfield(state.acq, ['acquiringChannel' num2str(channelCounter)]) % If acquiring..
%             if first % if its the first frame of first channel, then overwrite...
%                 imwrite(state.acq.acquiredData{channelCounter}(:,:,frameCounter) ... % BSMOD 1/18/2
%                     , fileName,  'WriteMode', 'overwrite', 'Compression', 'none', 'Description', state.headerString);
%                 first = 0;
%             elseif ~all(all(state.acq.acquiredData{channelCounter}(:,:,frameCounter)==0))
%                 imwrite(state.acq.acquiredData{channelCounter}(:,:,frameCounter) ... % BSMOD 1/18/2
%                     , fileName,  'WriteMode', 'append', 'Compression', 'none');
%             end
%         end
%     end
% end
%
%disp(['File ' fileName ' saved.']); %VI091508B
%setStatusString(stat);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

