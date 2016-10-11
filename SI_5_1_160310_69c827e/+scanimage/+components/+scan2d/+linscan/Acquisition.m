classdef Acquisition < scanimage.interfaces.Class
    %% Class specific properties            
    %     properties (Dependent)
    %         channelsInputRanges;                    % [V] 1D cell array of [min max] input ranges for each channel
    %     end
    
    properties
        startTrigIn = '';
        startTrigEdge = 'rising';
    end
    
    %% Internal properties
    properties (SetAccess = immutable)
        hLinScan;            					% handle of hLinScan
    end
    
    properties (Constant)
        ACQ_BUFFER_SIZE = 30;                   % size of the AI input buffer in stripes
    end
    
    properties (SetAccess = private)
        hAI;                                    % handle of AI Task for digitizing light input during scan, e.g. a PMT signal
        hAIOnDemand;                            % AI Task for acquiring single samples from the PMTs (i.e. for reading dark offsets)
        stripeCounter = 0;                      % total number of stripes acquired
        frameCounter = 0;                       % total number of frames acquired
        everyNSamples;                          % zzSamplesAcquiredFcn is called every N samples
        acqParamBuffer = struct();              % buffer holding frequently used parameters to limit parameter recomputation
        sampleBuffer = scanimage.components.scan2d.linscan.SampleBuffer(); % buffer holding samples for stripes
        acqDevType;
        endOfAcquisition = false;
    end
    
    properties (Dependent, SetAccess = private)
        active;                                 % (logical) ndicates if the current task is active
    end
    
    %% Lifecycle
    methods
        function obj = Acquisition(hLinScan)
            obj.hLinScan = hLinScan;
            
            obj.ziniPrepareTasks();
            obj.ziniExpandChannelsInvert();
        end
        
        function delete(obj)
            obj.zprvClearTask('hAI');
            obj.zprvClearTask('hAIOnDemand');
        end
    end
    
    %% Public Methods
    methods
        function start(obj)
            obj.assertNotActive('method:start');
            
            obj.bufferAcqParams();
            
            % reset counters
            obj.stripeCounter = 0;
            obj.frameCounter = 0;
            obj.endOfAcquisition = false;
            
            % configure AI task for acquisition
            obj.zzConfigSampModeAndSampPerChan();
            obj.zzConfigInputEveryNAndBuffering();
            obj.sampleBuffer.initialize(obj.acqParamBuffer.samplesPerFrame,length(obj.hAI.channels),obj.hLinScan.channelsDataType);
            
            obj.hAI.start();
        end
        
        function updateBufferedPhaseSamples(obj)
            obj.acqParamBuffer.linePhaseSamples = round(obj.hLinScan.linePhase * obj.hLinScan.sampleRate); % round to avoid floating point accuracy issue
        end
        
        function updateBufferedOffsets(obj)
            if ~isempty(obj.acqParamBuffer)
                tmpValA = cast(obj.hLinScan.hSI.hChannels.channelOffset(obj.acqParamBuffer.channelsActive),obj.hLinScan.channelsDataType);
                tmpValB = cast(obj.hLinScan.hSI.hChannels.channelSubtractOffset(obj.acqParamBuffer.channelsActive),obj.hLinScan.channelsDataType);
                channelsOffset = tmpValA .* tmpValB;
                obj.acqParamBuffer.channelsOffset = channelsOffset .* cast(obj.hLinScan.pixelBinFactor,obj.hLinScan.channelsDataType);
            end
        end
        
        function bufferAcqParams(obj,live)
            if nargin < 2 || isempty(live) || ~live
                obj.acqParamBuffer = struct(); % flush buffer
            end
            
            roiGroup = obj.hLinScan.currentRoiGroupScannerCoords;
            scannerset=obj.hLinScan.scannerset;
            zs=obj.hLinScan.hSI.hStackManager.zs; % generate slices to scan based on motor position etc
            
            obj.acqParamBuffer.roi = roiGroup.rois;
            obj.acqParamBuffer.scanField = roiGroup.rois.scanfields;
            
            [lineScanPeriod, lineAcqPeriod] = scannerset.linePeriod(obj.acqParamBuffer.scanField);
            obj.acqParamBuffer.scanFieldParams =  struct('lineScanSamples',round(lineScanPeriod * obj.hLinScan.sampleRate),...
                'lineAcqSamples',round(lineAcqPeriod * obj.hLinScan.sampleRate),...
                'pixelResolution',obj.acqParamBuffer.scanField.pixelResolution);
            
            if nargin < 2 || isempty(live) || ~live
                fbZs = obj.hLinScan.hSI.hFastZ.numDiscardFlybackFrames;
                times = arrayfun(@(z)roiGroup.sliceTime(scannerset,z),zs);
                obj.acqParamBuffer.frameTime  = max(times);
                
                [obj.acqParamBuffer.startSample,obj.acqParamBuffer.endSample] = roiSamplePositions(roiGroup,scannerset,0);
                
                obj.acqParamBuffer.scannerset = scannerset;
                obj.acqParamBuffer.zs = zs;
                obj.acqParamBuffer.flybackFramesPerStack = fbZs;
                obj.acqParamBuffer.numSlices  = numel(zs);
                obj.acqParamBuffer.roiGroup = roiGroup;
                
                obj.updateBufferedPhaseSamples();
                
                lclChannelsActive = obj.hLinScan.hSI.hChannels.channelsActive;
                obj.acqParamBuffer.channelsActive = lclChannelsActive;
                obj.acqParamBuffer.channelsSign = cast(1 - 2*obj.hLinScan.mdfData.channelsInvert(lclChannelsActive),obj.hLinScan.channelsDataType); % -1 for obj.mdfData.channelsInvert == true, 1 for obj.mdfDatachannelsInvert == false
            
                obj.updateBufferedOffsets();
            end
            
            function [startSamples, endSamples] = roiSamplePositions(roiGroup,scannerset,z)
                % for each roi at z, determine the start and end time
                transitTimes = reshape(roiGroup.transitTimes(scannerset,z),1,[]); % ensure these are row vectors
                scanTimes    = reshape(roiGroup.scanTimes(scannerset,z),1,[]);
                
                % timeStep = 1/scannerset.sampleRateHz;
                times = reshape([transitTimes;scanTimes],1,[]); % interleave transit Times and scanTimes
                times = cumsum(times);  % cumulative sum of times
                times = reshape(times,2,[]);    % reshape to separate start and stop time vectors
                startTimes = times(1,:);
                endTimes   = times(2,:);
                
                startSamples = arrayfun(@(x)(round(x * obj.hLinScan.sampleRate) + 1), startTimes); % increment because Matlab indexing is 1-based
                endSamples   = arrayfun(@(x)(round(x * obj.hLinScan.sampleRate)),endTimes );
            end
        end
        
        function restart(obj)
            % reset counters
            obj.stripeCounter = 0;
            obj.frameCounter = 0;
            obj.endOfAcquisition = false;
            
            obj.assertNotActive('method:restart');
            
            % TODO: should the acquisition buffer be flushed here?
            obj.hAI.start();
        end
        
        function abort(obj)
            try
                obj.hAI.abort();
                obj.hAI.control('DAQmx_Val_Task_Unreserve');
            catch ME
                most.idioms.reportError(ME);
            end
        end
        
        function data = acquireSamples(obj,numSamples)
            obj.assertNotActive('acquireSamples');
            obj.hAI.control('DAQmx_Val_Task_Unreserve');
            data = obj.hAIOnDemand.readAnalogData(numSamples, 'native', 10);
            
            channelsSign = 1 - 2*obj.hLinScan.mdfData.channelsInvert; % -1 for obj.mdfData.channelsInvert == true, 1 for obj.mdfDatachannelsInvert == false
            for chan = 1:size(data,2)
                data(:,chan) = data(:,chan) * channelsSign(chan);     % invert channels
            end
        end
    end
    
    %% Private Methods
    methods (Access = private)
        function ziniPrepareTasks(obj)
            import dabs.ni.daqmx.*;
            hDev = dabs.ni.daqmx.Device(obj.hLinScan.mdfData.deviceNameAcq);
            obj.acqDevType = hDev.productCategory;
            
            %Initialize hAI Tasks
            obj.hAI = most.util.safeCreateTask([obj.hLinScan.name '-AnalogInput']);
            obj.hAIOnDemand = most.util.safeCreateTask([obj.hLinScan.name '-AnalogInputOnDemand']);
            
            % make sure not more channels are created then there are channels available on the device
            for i=1:obj.hLinScan.channelsAvailable
                obj.hAI.createAIVoltageChan(obj.hLinScan.mdfData.deviceNameAcq,obj.hLinScan.mdfData.channelIDs(i),sprintf('Imaging-%.2d',i-1),-1,1);
                obj.hAIOnDemand.createAIVoltageChan(obj.hLinScan.mdfData.deviceNameAcq,obj.hLinScan.mdfData.channelIDs(i),sprintf('ImagingOnDemand-%.2d',i-1));
            end
            
            % the AI task reuses the sample clock of the AO task this
            % guarantees the two tasks start at the same time and stay in sync
            obj.hAI.cfgSampClkTiming(obj.hAI.get('sampClkMaxRate'), 'DAQmx_Val_FiniteSamps', 2);
            obj.hAI.everyNSamplesReadDataEnable = true;
            obj.hAI.everyNSamplesReadDataTypeOption = 'native';
            obj.hAI.everyNSamplesEventCallbacks = @(src,evnt)obj.zzSamplesAcquiredFcn(src,evnt);
            obj.hAI.doneEventCallbacks = @(src,evt)obj.zzDoneEventFcn(src,evt);
            
            % the on demand AI task does not use a sample clock
            obj.hAIOnDemand.everyNSamplesReadDataTypeOption = 'native';
        end
        
        function zzDoneEventFcn(obj,~,~)
            % when the event rate is high, for some strange reason the last
            % everyNSamples event of a finite acquisition is not fired, but
            % the doneEvent is fired instead. To work around this issue,
            % register both callbacks. if the done event is fired, generate
            % a 'pseudo callback' for the everyNSamples event
            availableSamples = obj.hAI.get('readAvailSampPerChan');
            if obj.everyNSamples == availableSamples;
                evt = struct();
                evt.data = obj.hAI.readAnalogData(availableSamples,'native',0);
                evt.errorMessage = [];
                obj.zzSamplesAcquiredFcn([],evt); % call the everNSamples callback with the pseudo event data
            elseif availableSamples ~= 0
                % this should never happen. if the done event is fired the
                % input buffer should be either empty, or the last frame
                % (availablesamples == obj.everyNSamples) should be in the
                % buffer
                most.idioms.reportError('LinScan Acq: Something bad happened: Available number of samples does not match expected number of samples.');
                obj.hLinScan.hSI.abort();
            end
        end
        
        function ziniExpandChannelsInvert(obj)
            if isscalar(obj.hLinScan.mdfData.channelsInvert)
                obj.hLinScan.mdfData.channelsInvert = repmat(obj.hLinScan.mdfData.channelsInvert,1,obj.hLinScan.channelsAvailable);
            else
                assert(isvector(obj.hLinScan.mdfData.channelsInvert));
                assert(numel(obj.hLinScan.mdfData.channelsInvert) >= obj.hLinScan.channelsAvailable,...
                    'MDF invalid setting: If providing a vector for MDF entry ''channelsInvert'', provide a value for each channel in the task.');
            end
        end
        
        function zzConfigInputEveryNAndBuffering(obj)
            %Determine everyNSamples value
            samplesPerFrame = round(obj.acqParamBuffer.frameTime * obj.hLinScan.sampleRate);
            obj.acqParamBuffer.samplesPerFrame = samplesPerFrame;
            obj.acqParamBuffer.numStripes = determineNumStripes(obj.acqParamBuffer,samplesPerFrame);
            
            obj.everyNSamples = round(samplesPerFrame / obj.acqParamBuffer.numStripes);
            bufferNumSamples = obj.ACQ_BUFFER_SIZE * obj.everyNSamples;
            
            %Apply everyNSamples & buffer values
            obj.hAI.everyNSamples = []; %unregisters callback
            obj.hAI.cfgInputBufferVerify(bufferNumSamples,2*obj.everyNSamples);
            obj.hAI.everyNSamples = obj.everyNSamples; %registers callback
            
            function numStripes = determineNumStripes(acqParamBuffer,samplesPerFrame)
                if obj.hLinScan.mdfData.stripingEnable ...
                        && length(acqParamBuffer.roiGroup.rois) == 1 ...
                        && length(acqParamBuffer.roiGroup.rois(1).scanfields) == 1
                    
                    maxNumStripes = acqParamBuffer.frameTime * obj.hLinScan.mdfData.stripingMaxRate;
                    possibleNumStripes = divisors(samplesPerFrame);
                    possibleNumStripes = possibleNumStripes(possibleNumStripes <= maxNumStripes);
                    numStripes = max(possibleNumStripes);
                    if isempty(numStripes)
                        numStripes = 1;
                    end
                else
                    numStripes = 1;
                end
            end
            
            function d = divisors(n) % local function
                % this algorithm should be sufficiently fast for small values of n
                d = 1:n/2;            % list of possible divisors
                d = d(mod(n,d) == 0); % test all possible divisors
            end
        end
        
        function zzConfigSampModeAndSampPerChan(obj,forceContinuous)
            if nargin < 2 || isempty(forceContinuous)
                forceContinuous = false;
            end
            
            if forceContinuous || obj.hLinScan.framesPerAcq <= 0 || isinf(obj.hLinScan.framesPerAcq) || obj.hLinScan.trigNextStopEnable
                obj.hAI.sampQuantSampMode = 'DAQmx_Val_ContSamps';
            else
                samplesPerFrame = round(obj.acqParamBuffer.frameTime * obj.hLinScan.sampleRate);
                obj.hAI.sampQuantSampMode = 'DAQmx_Val_FiniteSamps';
                
                numSamples = samplesPerFrame * obj.hLinScan.framesPerAcq;
                
                if numSamples > 16777213 && strcmpi(obj.acqDevType,'DAQmx_Val_SSeriesDAQ');
                    %limitation in legacy S-Series (e.g. 6110): cannot set
                    %sampQuantSampPerChan to a high value, use continuous
                    %mode instead
                    obj.zzConfigSampModeAndSampPerChan(true);
                elseif numSamples >= 2^32
                    obj.zzConfigSampModeAndSampPerChan(true);
                else
                    % DAQmx property sampQuantSampPerChan is limited to 2^32-1
                    assert(numSamples < 2^32,['Exceeded maximum number of frames per acquisition.\n' ...
                    'Requested: %d; Maximum possible with current settings: %d (=%d min acquisition time) \n' ...
                    'Workaround: set number of frames to Inf (number of volumes for FastZ acquisition)'],...
                    obj.hLinScan.framesPerAcq,floor((2^32-1)/samplesPerFrame),round((2^32-1)/(60*obj.hLinScan.sampleRate)));
                
                    obj.hAI.sampQuantSampPerChan = numSamples;
                end
            end
        end
        
        function zzSamplesAcquiredFcn(obj,~,event)
            if obj.endOfAcquisition
                return
            end
            
            startProcessingTime = tic;            
            % get raw data from task            
            inputSamples = event.data;
            
            if ~isempty(event.errorMessage)                
                most.idioms.reportError('Error during readAnalogData(): \t%s\n',event.errorMessage);
                obj.hLinScan.hSI.abort();
            end
            
            if size(inputSamples,1) ~= obj.everyNSamples
                most.idioms.reportError('Did not receive expected number of samples from analog input task');
                obj.hLinScan.hSI.abort();
            end
            
            % calculate local frame and stripe number
            % this needs to be done before the object counters are updated!
            frameNumber = obj.frameCounter + 1;
            stripeNumber = mod(obj.stripeCounter, obj.acqParamBuffer.numStripes) + 1;
            
            % update stripe and frame counter
            obj.stripeCounter = obj.stripeCounter + 1;
            if ~mod(obj.stripeCounter,obj.acqParamBuffer.numStripes)
                obj.frameCounter = obj.frameCounter + 1;
            end
            
            if obj.frameCounter >= obj.hLinScan.framesPerAcq && obj.hLinScan.framesPerAcq > 0 && ~obj.hLinScan.trigNextStopEnable
                obj.endOfAcquisition = true;
            end
            
            % construct stripe data object
            stripeData = scanimage.interfaces.StripeData();
            stripeData.frameNumberAcq = frameNumber;
            stripeData.stripeNumber = stripeNumber;
            stripeData.stripesRemaining = 0;
            stripeData.startOfFrame = (stripeNumber == 1);
            stripeData.endOfFrame = (stripeNumber == obj.acqParamBuffer.numStripes);
            stripeData.endOfAcquisition = obj.endOfAcquisition;
            stripeData.overvoltage = false; % TODO: check for overvoltage
            stripeData.channelNumbers = obj.hLinScan.hSI.hChannels.channelsActive;
            
            stripeData = obj.zzDataToRois(stripeData,inputSamples);
            % stripe data is still transposed at this point
            obj.hLinScan.zzStripeAcquiredCallback(stripeData, startProcessingTime);
        end
        
        function stripeData = zzDataToRois(obj,stripeData,ai)   
            if stripeData.startOfFrame;
                obj.sampleBuffer.reset();
            end
            
            obj.sampleBuffer.appendData(ai);
            
            zs = obj.acqParamBuffer.zs;
            if length(zs) > 1
                flybackFrames = obj.acqParamBuffer.flybackFramesPerStack;
            else
                flybackFrames = 0;
            end
            
            APB = obj.acqParamBuffer;
            numSlices = APB.numSlices;
            sliceNum = mod(stripeData.frameNumberAcq-1+flybackFrames,numSlices)+1;
            
            if sliceNum > numSlices
                % flyback frame
                stripeData.roiData = {};
            else
                stripeData.roiData = {};

                scannerset = APB.scannerset;
                z = zs(sliceNum);
                scanFieldParams = APB.scanFieldParams;
                fieldSamples    = [APB.startSample APB.endSample];
                roi             = APB.roi;
                
                if ~isempty(roi)
                    [success,imageDatas,stripePosition] = scannerset.formImage(scanFieldParams,obj.sampleBuffer,fieldSamples,APB.channelsActive,APB.linePhaseSamples);
                    
                    if success
                        roiData = scanimage.mroi.RoiData;
                        roiData.hRoi = roi;
                        roiData.zs = z;
                        roiData.stripePosition = {stripePosition};
                        roiData.stripeFullFrameNumLines = scanFieldParams.pixelResolution(2);
                        roiData.channels = APB.channelsActive;
                        for iter = 1:length(imageDatas)
                            image = imageDatas{iter} .* APB.channelsSign(iter) - APB.channelsOffset(iter);
                            
                            if APB.numStripes > 1
                                roiData.imageData{iter}{1} = zeros(scanFieldParams.pixelResolution(1),scanFieldParams.pixelResolution(2));
                                roiData.imageData{iter}{1}(:,stripePosition(1):stripePosition(2)) = image;
                            else
                                roiData.imageData{iter}{1} = image;
                            end
                        end
                        stripeData.roiData{1} = roiData;
                    end
                end
            end
        end
    end
    
    %% Setter/Getter Methods
    methods        
        function val = get.active(obj)
            val = ~obj.hAI.isTaskDoneQuiet();
        end
        
        function set.startTrigIn(obj,val)
            obj.assertNotActive('startTrigIn');
            
            switch obj.startTrigEdge
                case 'rising'
                    edge = 'DAQmx_Val_Rising';
                case 'falling'
                    edge = 'DAQmx_Val_Falling';
                otherwise
                    assert(false);
            end
            
            if isempty(val)
                obj.hAI.disableStartTrig();
            else
                obj.hAI.cfgDigEdgeStartTrig(val,edge);
            end
        end
        
        function set.startTrigEdge(obj,val)
            obj.assertNotActive('startTrigEdge');
            assert(ismember(val,{'rising','falling'}));
            obj.startTrigEdge = val;
            obj.startTrigIn = obj.startTrigIn;
        end
        
        %         function val = get.channelsInputRanges(obj)
        %             numChans = numel(obj.hAI.channels);
        %             ranges = cell(1,numChans);
        %             for i = 1:numChans
        %                 hAIChan = obj.hAI.channels(i);
        %                 min = get(hAIChan,'min');
        %                 max = get(hAIChan,'max');
        %                 ranges{i} = [min, max];
        %             end
        %             val = ranges;
        %         end
        %
        %         function set.channelsInputRanges(obj,val)
        %             obj.assertNotActive('channelsInputRanges');
        %             validateattributes(val,{'cell'},{});
        %
        %             numInput = numel(val);
        %             numChans = numel(obj.hAI.channels);
        %             assert(numInput <= numChans);
        %
        %             % in case numInput < numChans
        %             inputRanges = obj.channelsInputRanges();
        %             inputRanges(1:numInput) = val(:);
        %
        %             % set the new inputRanges
        %             for i = 1:numel(inputRanges)
        %                 inputRange = inputRanges{i};
        %                 validateattributes(inputRange,{'numeric'},{'vector','numel',2});
        %
        %                 hAIchan = obj.hAI.channels(i);
        %                 hAIchan.min = inputRange(1);
        %                 hAIchan.max = inputRange(2);
        %             end
        %         end
    end
    
    %% Helper functions
    methods (Access = private)        
        function assertNotActive(obj,propName)
            assert(~obj.active,'Cannot access property %s during an active acquisition',propName);
        end
        
        function valCoercedWarning(~,propName,requestedVal,actualVal)
            if requestedVal ~= actualVal
                warning('%s was coerced to the nearest possible value. Requested: %d Actual: %d', ...
                    propName, requestedVal, actualVal);
            end
        end
        
        function zprvClearTask(obj, taskPropertyName)
            hTask = obj.(taskPropertyName);
            
            if isempty(hTask) || ~isvalid(hTask)
                return;
            end
            
            hTask.clear();
            obj.(taskPropertyName) = [];
        end
    end
end


%--------------------------------------------------------------------------%
% Acquisition.m                                                            %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
