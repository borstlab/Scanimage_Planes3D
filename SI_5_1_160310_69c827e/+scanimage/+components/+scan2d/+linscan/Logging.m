classdef Logging < scanimage.interfaces.Class
    properties (SetAccess = private, Hidden)
        hLinScan;
        hTifs;
        
        active = false;
        
        fileCounter;
        fileFrameCounter;
        fileSubCounter;
        
        blankFrameDescription;
        
        stripeBuffer = [];
        
        mRoiLogging = false;
        linesPerFrame;
        pixelsPerLine;
    end
    
    properties (Dependent, SetAccess = private)
       bitsPerSample;
       dataSigned;
    end
    
    properties (Constant)
        FRAME_DESCRIPTION_LENGTH = 2^14-1; % same value as in ResScan       
    end
    
    %% Lifecycle
    methods
        function obj = Logging(hLinScan)
            obj.hLinScan = hLinScan;
            obj.blankFrameDescription = repmat(' ',1,obj.FRAME_DESCRIPTION_LENGTH);
        end
        
        function delete(obj)
            obj.abort();
            obj.closeFiles(); % forces all open file handles to be closed
        end
    end
    
    methods
        function start(obj)
            if ~obj.hLinScan.hSI.hChannels.loggingEnable;return;end
            if isempty(obj.hLinScan.hSI.hChannels.channelSave);return;end
            
            obj.closeFiles();
            
            obj.stripeBuffer = [];
            
            obj.fileCounter = obj.hLinScan.logFileCounter;
            obj.fileFrameCounter = 0;
            obj.fileSubCounter = 0;
            
            zs=obj.hLinScan.hSI.hStackManager.zs; % generate planes to scan based on motor position etc
            roiGroup = obj.hLinScan.currentRoiGroupScannerCoords;
            scanFields = arrayfun(@(z)roiGroup.scanFieldsAtZScannerCoordinates(obj.hLinScan.scannerset,z),...
                                  zs,'UniformOutput',false);
            
            obj.mRoiLogging = false;
            cumPixelResolutionAtZ = zeros(0,2);
            for zidx = 1:length(scanFields)
                sfs = scanFields{zidx};
                pxRes = zeros(0,2);
                for sfidx = 1:length(sfs)
                    sf = sfs{sfidx};
                    pxRes(end+1,:) = sf.pixelResolution(:)'; 
                end
                obj.mRoiLogging = obj.mRoiLogging || size(pxRes,1) > 1;
                cumPixelResolutionAtZ(end+1,:) = [max(pxRes(:,1)), sum(pxRes(:,2))];
            end
            
            obj.mRoiLogging = obj.mRoiLogging || ~unique(cumPixelResolutionAtZ(:,1)) || ~unique(cumPixelResolutionAtZ(:,2));
            obj.linesPerFrame = max(cumPixelResolutionAtZ(:,2));
            disp('Logging, linesPerFrame')
            disp(obj.linesPerFrame)
            obj.pixelsPerLine = max(cumPixelResolutionAtZ(:,1));
                        
            % create TifStream objects
            numChannels = length(obj.hLinScan.hSI.hChannels.channelSave);
            if obj.hLinScan.logFilePerChannel
                obj.hTifs = cell(1,numChannels);
                for i = 1:numChannels
                    chan = obj.hLinScan.hSI.hChannels.channelSave(i);
                    disp('creating TIfStream objects, linePerFrame')
                    disp(obj.linesPerFrame)
                    obj.hTifs{i} = scanimage.components.scan2d.linscan.TifStream(obj.makeFullFilePath(chan),...
                        obj.pixelsPerLine, obj.linesPerFrame, obj.blankFrameDescription,...
                        'dataType',obj.hLinScan.channelsDataType,'overwrite',true);
                end
            else
                obj.hTifs = cell(1,1);
                disp('creating TIfStream objects, linePerFrame')
                disp(obj.linesPerFrame)
                obj.hTifs{1} = scanimage.components.scan2d.linscan.TifStream(obj.makeFullFilePath,...
                    obj.pixelsPerLine, obj.linesPerFrame, obj.blankFrameDescription,...
                    'dataType',obj.hLinScan.channelsDataType,'overwrite',true); 
            end
            
            obj.active = true;
        end
        
        function logStripe(obj,stripeData)            
            if ~obj.hLinScan.hSI.hChannels.loggingEnable;return;end
            if isempty(obj.hLinScan.hSI.hChannels.channelSave);return;end
            
            if stripeData.startOfFrame && stripeData.endOfFrame
                obj.stripeToDisk(stripeData);
            else
                assert(~obj.mRoiLogging,'Something bad happened: trying to save a partial frame (''stripe'') while logging mroi data. This is not allowed.');
                % striped frame coming in
                if stripeData.startOfFrame
                    obj.stripeBuffer = copy(stripeData);
                else
                    newStripe = copy(stripeData); % memory copy of entire frame for every stripe -> not good for performance
                    newStripe.merge(obj.stripeBuffer);
                    obj.stripeBuffer = newStripe;
                end
                
                if stripeData.endOfFrame
                    obj.stripeToDisk(obj.stripeBuffer);
                end
            end      
        end
        
        function stripeToDisk(obj,stripeData)
            % write frames to disk
            obj.fileFrameCounter = obj.fileFrameCounter + 1;
            frameDescription = sprintf('%s\n%s\n',stripeData.frameDescription,obj.hLinScan.logHeaderString);

            numChannels = length(obj.hLinScan.hSI.hChannels.channelSave);
            for i = 1:numChannels
                chanNum = obj.hLinScan.hSI.hChannels.channelSave(i);

                if obj.hLinScan.logFilePerChannel
                    fileIndex = i;
                else
                    fileIndex = 1;
                end

                chIdx = find(stripeData.roiData{1}.channels == chanNum,1,'first');
                obj.hTifs{fileIndex}.imageDescription = frameDescription;
                
                if obj.mRoiLogging
                    line = 1;
                    tempbuf = zeros(obj.pixelsPerLine,obj.linesPerFrame,obj.hLinScan.channelsDataType);
                    disp('SIZE OF tempbuf')
                    disp(size(tempbuf))
                    for roiIdx = 1:length(stripeData.roiData)
                        imdata = stripeData.roiData{roiIdx}.imageData{chIdx}{1};
                        dims = size(imdata);
                        disp('dims of imdata')
                        disp(dims)
                        tempbuf(1:dims(1),line:line+dims(2)-1) = imdata;
                        line = line + dims(2);
                    end
                    obj.hTifs{fileIndex}.appendFrame(tempbuf,true);
                else
                    obj.hTifs{fileIndex}.appendFrame(stripeData.roiData{1}.imageData{chIdx}{1},true);
                end
            end

            % determine if file is split after this frame
            newFileFlag = false;

            if obj.fileFrameCounter >= obj.hLinScan.logFramesPerFile
                obj.fileSubCounter = obj.fileSubCounter + 1;
                newFileFlag = true;
            end

            if stripeData.endOfAcquisition && ~stripeData.endOfAcquisitionMode
                obj.fileCounter = obj.fileCounter + 1;
                obj.fileSubCounter = 0;
                newFileFlag = true;
            end

            if newFileFlag
                obj.newFile();
            end
        end
        
        function newFile(obj)
            if ~obj.hLinScan.hSI.hChannels.loggingEnable;return;end
            if isempty(obj.hLinScan.hSI.hChannels.channelSave);return;end
            
            obj.fileFrameCounter = 0;
            
            numChannels = length(obj.hLinScan.hSI.hChannels.channelSave);
            if obj.hLinScan.logFilePerChannel
                for i = 1:numChannels
                    chan = obj.hLinScan.hSI.hChannels.channelSave(i);
                    oldfilename = obj.hTifs{i}.newFile(obj.makeFullFilePath(chan));
                    obj.appendRoiGroupData(oldfilename);
                end
            else
                obj.hTifs{1}.newFile(obj.makeFullFilePath());
            end
        end
        
        function abort(obj)            
            obj.closeFiles();
            obj.active = false;
        end
    end
    
    methods (Access = private)
        function closeFiles(obj)
            if ~isempty(obj.hTifs)
                for i = 1:length(obj.hTifs)
                    try
                        hTif = obj.hTifs{i};
                        if ~isempty(hTif) && isvalid(hTif)
                            oldfilename = hTif.close();
                            obj.appendRoiGroupData(oldfilename);
                        end
                    catch ME
                        most.idioms.reportError(ME);
                    end
                end
            end
            obj.hTifs = {};
        end
        
        function fullPath = makeFullFilePath(obj,channelNum)
            if nargin < 2 || isempty(channel)
                channelNum = [];
            end
            
            stringFileCounter = sprintf('_%0u',obj.fileCounter);
            
            if isinf(obj.hLinScan.logFramesPerFile)
                stringFileSubCounter = '';
            else
                stringFileSubCounter = sprintf('_%0u',obj.fileSubCounter);
            end
            
            if isempty(channelNum)
                stringChannelNum = '';
            else
                stringChannelNum = sprintf('_chn%u',channelNum);
            end
            
            fileName = [obj.hLinScan.logFileStem stringFileCounter stringFileSubCounter stringChannelNum];
            fullPath = fullfile(obj.hLinScan.logFilePath,fileName); % extension is automatically appended by TifStream
        end
        
        function appendRoiGroupData(obj,filename)
            if isempty(filename) || isempty(obj.hLinScan.appendDataStringForMex)
                return
            end
            
            fid = fopen(filename,'a');
            fwrite(fid,obj.hLinScan.appendDataStringForMex);
            fwrite(fid,length(obj.hLinScan.appendDataStringForMex),'uint32');
            fclose(fid);
        end
    end
end



%--------------------------------------------------------------------------%
% Logging.m                                                                %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
