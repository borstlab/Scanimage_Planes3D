classdef LinScan < scanimage.components.Scan2D & most.HasMachineDataFile    
    %% ABSTRACT PROPERTY REALIZATION (most.Model)
    properties (Hidden, SetAccess=protected)
        mdlPropAttributes = zlclAppendDependsOnPropAttributes(scanimage.components.Scan2D.scan2DPropAttributes());
        mdlHeaderExcludeProps = {};
    end
    
    %% Abstract property realizations (most.HasMachineDataFile)
    properties (Constant, Hidden)
        %Value-Required properties
        mdfClassName = mfilename('class');
        mdfHeading = 'LinScan';
        
        %Value-Optional properties
        mdfDependsOnClasses; %#ok<MCCPI>
        mdfDirectProp; %#ok<MCCPI>
        mdfPropPrefix; %#ok<MCCPI>
        
        mdfOptionalVars = struct(...
            'stripingEnable', true,...      % enables/disables striping display
            'stripingMaxRate', 10,...       % [Hz] determines the maximum display update rate for striping
            'maxDisplayRate', 30,...        % [Hz] limits the maximum display rate (affects frame batching)
            ...
            'internalRefClockSrc', [],...   % Reference clock to use internally
            'internalRefClockRate', [],...  % Rate of reference clock to use internally
            'enableRefClkOutput', false... % Enables/disables the export of the 10MHz reference clock on PFI14
			);
    end
    
    %% ABSTRACT PROPERTY REALIZATION (scanimage.interfaces.Component)
    properties (SetAccess = protected, Hidden)
        numInstances = 0;
    end
    
    properties (Constant, Hidden)
        COMPONENT_NAME = 'LinScan';                  % [char array] short name describing functionality of component e.g. 'Beams' or 'FastZ'
    
        PROP_TRUE_LIVE_UPDATE = {'linePhase','beamClockDelay','logFileCounter'};        % Cell array of strings specifying properties that can be set while the component is active
        PROP_FOCUS_TRUE_LIVE_UPDATE = {};   % Cell array of strings specifying properties that can be set while focusing
        DENY_PROP_LIVE_UPDATE = {'framesPerAcq','framesPerStack','trigAcqTypeExternal',...   % Cell array of strings specifying properties for which a live update is denied (during acqState = Focus)
            'trigAcqTypeExternal','trigNextStopEnable','trigAcqInTerm',...
            'trigNextInTerm','trigStopInTerm','trigAcqEdge','trigNextEdge',...
            'trigStopEdge','stripeAcquiredCallback'...
            'logAverageFactor','logFilePath',...
            'logFileStem','logFramesPerFile','logFramesPerFileLock',...
            'logHeaderString','logNumSlices'};
        
        FUNC_TRUE_LIVE_EXECUTION = {'readStripeData','trigIssueSoftwareAcq','updateLiveValues',...
            'trigIssueSoftwareNext','trigIssueSoftwareStop','measureScannerFrequency'};  % Cell array of strings specifying functions that can be executed while the component is active
        FUNC_FOCUS_TRUE_LIVE_EXECUTION = {}; % Cell array of strings specifying functions that can be executed while focusing
        DENY_FUNC_LIVE_EXECUTION = {'centerScanner','pointScanner','parkScanner','acquireSamples'}; % Cell array of strings specifying functions for which a live execution is denied (during acqState = Focus)
    end
    
    %% Abstract property realizations (scanimage.subystems.Scan2D)
    properties (Constant)
        scannerType = 'Linear';
    end
    
    properties (SetObservable)
        pixelBinFactor = 4;                 % number of acquisition samples that form one pixel, only applicable in LinScan
        sampleRate = 1.25e6;                % [Hz] sample rate of the digitizer / mirror controls
    end
    
    properties (SetObservable, Transient)
        channelOffsets;
    end
    
    properties (SetObservable, Hidden)
        keepResonantScannerOn;
    end
    
    properties (Hidden, Dependent, SetAccess = protected, Transient) % SI internal properties, not SetObservable
        linePhaseStep;                       % [s] minimum step size of the linephase
    end
    
    properties (Hidden, SetAccess = protected)
        defaultRoiFovSize;
        angularRange;
        supportsRoiRotation = true;
    end
    
    %% Class specific properties
    properties (Hidden)
        logFilePerChannel = false;           % boolean, if true, each channel is saved to a separate file
    end
    
    properties (Constant, Hidden)
        MAX_NUM_CHANNELS = 4;               % Maximum number of channels supported
        MAX_REQUESTED_CTL_RATE = 500e3;     % [Hz] if acquisition sample rate and galvo output rate are independent, limit the galvo output rate to this value
    end
    
    properties (Hidden, SetAccess = private)
        maxSampleRateCtl;                   % [Hz] maximum sample rate achievable by the XY Galvo control task
    end
    
    properties (Hidden, SetAccess = immutable)
        hAcq;                               % handle to image acquisition system
        hCtl;                               % handle to galvo control system
        hTrig;                              % handle to trigger system
        hLinScanLog;                        % handle to logging system
    end
    
    properties (Hidden, SetAccess = private)
        clockMaster;                        % {'auxiliary','controller'} specifies which board generates the sample clock/controlls the triggering
        linkedSampleClkAcqCtl;           % logical, indicates if the Acquisition AI and the Controller AO use the same sample clock
        sampleRateCtl;                      % [Hz] sample rate of the XY Galvo control task
        
        epochAcqMode;                       % software timestamp taken at the first captured stripe in the acqMode
        epochAcq;                           % [s] time difference between epochAcqMode and start of current acquistion
        acqCounter = 0;                     % number of finished acquisitions since start of acquisition mode    
        frameCounter = 0;                   % frames acquired since start of acquisition mode
        lastAcquiredFrame;                  % buffers the last acquired frame
        lastDisplayTic = tic();             % last time (tic) when the frame was sent to ScanImage for display
        
        trigStartSoftwareTimestamp;         
        trigNextSoftwareTimestamp;
        trigStopSoftwareTimestamp;
        
        % property bufferes
        channelsAvailable_;
        channelsDataType_;
    end
    
    %% Lifecycle
    methods
        function obj = LinScan(hSI, simulated, name, legacymode)
            
            if nargin < 2 || isempty(simulated)
                simulated = false;
            end
            
            if nargin < 3 || isempty(name)
                name = 'Linear';
            end
            
            if nargin > 3 && ~isempty(legacymode) && legacymode
                custMdfHeading = 'LinScan';
            else
                legacymode = false;
                custMdfHeading = ['LinScan (' name ')'];
            end
            
            obj = obj@scanimage.components.Scan2D(hSI,simulated,name,legacymode);
            obj = obj@most.HasMachineDataFile(true, custMdfHeading);
            
            assert(isempty(obj.mdfData.beamDaqID) || (obj.mdfData.beamDaqID <= obj.hSI.hBeams.numInstances), 'LinScan: Invalid value for beamDaqID');
            
            if isempty(obj.mdfData.deviceNameGalvo)
                disp('LinScan: ''deviceNameGalvo'' not specified. Trying to set up galvo output on digitizer board');
                obj.mdfData.deviceNameGalvo = obj.mdfData.deviceNameAcq;
            end
            
            if isempty(obj.mdfData.channelIDs)
                obj.mdfData.channelIDs = 0:3;
            end
            obj.zprvMDFVerify('channelIDs',{{'numeric'},{'integer' 'vector' 'nonnegative'}},[]);
            
            obj.hCtl = scanimage.components.scan2d.linscan.Control(obj);
            obj.hAcq = scanimage.components.scan2d.linscan.Acquisition(obj);
            obj.hTrig = scanimage.components.scan2d.linscan.Triggering(obj);
            obj.hLinScanLog = scanimage.components.scan2d.linscan.Logging(obj);

            obj.numInstances = 1;
            
            obj.maxSampleRateCtl = obj.hCtl.hAO.get('sampClkMaxRate');
            
            obj.ziniConfigureRouting();
            obj.sampleRate = min(obj.sampleRate,obj.maxSampleRate); % Synchronize hAcq and hCtl
                        
            %Initialize Scan2D props (not initialized by superclass)
            obj.channelsInputRanges = repmat(obj.channelsAvailableInputRanges(1),1,obj.channelsAvailable);
            obj.channelOffsets = zeros(1, obj.channelsAvailable);
            obj.channelsSubtractOffsets = true(1, obj.channelsAvailable);
        end
        
        function delete(obj) 
            obj.hAcq.delete();
            obj.hCtl.delete();
            obj.hTrig.delete();
            obj.hLinScanLog.delete();
        end
    end
    
    methods (Access = protected, Hidden)
        function componentStart(obj)
            assert(~obj.robotMode);
            obj.independentComponent = false;
            
            obj.epochAcq = 0;
            obj.acqCounter = 0;
            obj.frameCounter = 0;
            obj.trigStartSoftwareTimestamp = NaN;
            obj.trigStopSoftwareTimestamp = NaN;
            obj.trigNextSoftwareTimestamp = NaN;
            
            obj.hLinScanLog.start();
            obj.hTrig.startTiming();
            
            obj.configureStartTrigger();
            if obj.trigAcqTypeExternal
                obj.startAcquisition();
            else
                % do not start the acquisition yet, wait for software trigger instead
            end
        end
        
        function componentAbort(obj,varargin)
            obj.haltAcquisition();
            obj.hLinScanLog.abort();
            obj.hTrig.abortTiming();
            obj.hCtl.parkOrPointLaser();
            obj.independentComponent = true;
        end
    end
    
    %% User API
    methods
        function centerScanner(obj)
            if obj.componentExecuteFunction('centerScanner')
                obj.hCtl.centerScanner();
            end
        end
        
        function pointScanner(obj,fastDeg,slowDeg)
            if obj.componentExecuteFunction('pointScanner',fastDeg,slowDeg)
                obj.hCtl.parkOrPointLaser([fastDeg,slowDeg]);
            end
        end
        
        function parkScanner(obj)
            if obj.componentExecuteFunction('parkScanner')
                obj.hCtl.parkOrPointLaser();
            end
        end
        
   
        function trigIssueSoftwareAcq(obj)
            if obj.componentExecuteFunction('trigIssueSoftwareAcq')
                if ~obj.active
                    obj.componentShortWarning('Cannot generate software trigger while acquisition is inactive');
                    return;
                end
                
                obj.trigStartSoftwareTimestamp = now();
                
                if obj.trigAcqTypeExternal
                    assert(obj.hTrig.enabled,'Cannot issue software external trigger without auxiliary board');
                    obj.generateTrigger(obj.mdfData.deviceNameAux,obj.trigAcqInTerm);
                else
                    if ~obj.hCtl.active
                        if obj.acqCounter == 0
                            obj.startAcquisition();
                        elseif ~obj.trigNextStopEnable
                            obj.restartAcquisition();
                        end
                    end
                end
            end
        end
        
        function trigIssueSoftwareNext(obj)
            if obj.componentExecuteFunction('trigIssueSoftwareNext')
                if ~obj.active
                    obj.componentShortWarning('Cannot generate software trigger while acquisition is inactive');
                    return;
                end
                assert(obj.hTrig.enabled,'Next triggering unavailable: no auxiliary board specified');
                obj.trigNextSoftwareTimestamp = now();
            end
        end
        
        function trigIssueSoftwareStop(obj)
            if obj.componentExecuteFunction('trigIssueSoftwareStop')
                if ~obj.active
                    obj.componentShortWarning('Cannot generate software trigger while acquisition is inactive');
                    return;
                end
                assert(obj.hTrig.enabled,'Next triggering unavailable: no auxiliary board specified');
                obj.trigStopSoftwareTimestamp = now();
            end
        end
        
        function measureScannerFrequency(obj)
            if obj.componentExecuteFunction('measureScannerFrequency')
                obj.componentShortWarning('Measuring resonant scanner frequency is unsupported in scanner type ''%s''.',obj.scannerType);
            end
        end
    end
    
    %% Friend API
    methods (Hidden)
        function reinitRoutes(obj)
            if obj.mdlInitialized
                %reset all devices
                daqs = unique({obj.mdfData.deviceNameAcq obj.mdfData.deviceNameGalvo obj.mdfData.deviceNameAux});
                daqs = daqs(~cellfun(@isempty,daqs));
                hDaqs = cellfun(@(x)dabs.ni.daqmx.Device(x), daqs, 'UniformOutput', false);
                cellfun(@(x)x.reset(), hDaqs);
                
                obj.hTrig.hRouteRegistry.reinitRoutes();
            end
        end
        
        function val = calibrateLinePhase(obj,chanIdx)
            imData = getImage();
            imData = imData{1}; % only use first roi to determine linephase
            [im1,im2] = deinterlaceImage(imData);
            [~,pixelPhase] = detectPixelOffset(im1,im2);
            samplePhase = obj.pixelBinFactor * pixelPhase;
            phaseOffset = samplePhase / obj.sampleRate;
            obj.linePhase = obj.linePhase - phaseOffset / 2;
                        
            function imData = getImage()
                %get image from every roi
                roiDatas = obj.hSI.hDisplay.lastFrame.roiData;
                for i = numel(roiDatas):-1:1
                    imData{i} = roiDatas{i}.imageData{chanIdx}{1};
                    if roiDatas{i}.transposed
                        imData{i} = imData{i}';
                    end                        
                end
            end
            
            function [im1, im2] = deinterlaceImage(im)
                im1 = im(1:2:end,:);
                im2 = im(2:2:end,:);
            end
            
            function [iOffset,jOffset] = detectPixelOffset(im1,im2)
                c = abs(most.mimics.xcorr2circ(im1,im2));
                cdim = size(c);
                [~,idx] = max(c(:));
                [i,j] = ind2sub(cdim,idx);
                iOffset = floor((cdim(1)/2))+1-i;
                jOffset = floor((cdim(2)/2))+1-j;
            end
        end
        
        function updateLiveValues(obj)
            if obj.active && obj.componentExecuteFunction('updateLiveValues')
                obj.hSI.zzzUpdateAO();
                obj.hCtl.updateAnalogBufferAsync();
                
                if strcmpi(obj.hSI.acqState,'focus')
                    obj.hAcq.bufferAcqParams(true);
                end
            end
        end
        
        function arm(obj)
            % nothing to do
        end
        
        function startAcquisition(obj)            
            obj.haltAcquisition();
            
            % start clock slaves before clock masters
            % hAcq is slave to hCtl is slave to hTrig
            obj.hAcq.start();
            obj.hCtl.start();
            obj.hTrig.start();
        end
        
        function haltAcquisition(obj)
            obj.hAcq.abort();
            obj.hTrig.abort();
            obj.hCtl.abort();
        end
        
        function restartAcquisition(obj)
            obj.haltAcquisition();
            
            % start clock slaves before clock masters
            % hAcq is slave to hCtl is slave to hTrig
            obj.hAcq.restart();
            obj.hCtl.restart();
            obj.hTrig.restart();
        end
        
        function signalReadyReceiveData(obj)
           % No op 
        end
        
        function [success,stripeData] = readStripeData(obj)
            % do not use componentExecuteFunction for performance
            %if obj.componentExecuteFunction('readStripeData')
                success = ~isempty(obj.lastAcquiredFrame);
                stripeData = obj.lastAcquiredFrame;
                obj.lastAcquiredFrame = [];
            %end
        end
        
        function range = resetAngularRange(obj)
            obj.xAngularRange = obj.mdfData.xGalvoAngularRange;
            obj.yAngularRange = obj.mdfData.yGalvoAngularRange;
            range = [obj.xAngularRange obj.yAngularRange];
        end
   
        function data = acquireSamples(obj,numSamples)
            if obj.componentExecuteFunction('acquireSamples',numSamples)
                    data = obj.hAcq.acquireSamples(numSamples);
            end
        end
        
        function zzStripeAcquiredCallback(obj,stripeData,startProcessingTime)
            if obj.frameCounter == 0 && stripeData.stripeNumber == 1
                stripesPerFrame = 1;
                obj.epochAcqMode = now - ((obj.hAcq.acqParamBuffer.frameTime/stripesPerFrame - toc(startProcessingTime)) / 86400); %the stripeAcquiredCallback happens _after_ the stripe is acquired, so subtract duration of stripe. 86400 = seconds per day
                obj.epochAcq = 0;
            end
            
            triggerTimes = obj.processTriggers(stripeData,startProcessingTime);
            
            if obj.trigNextStopEnable
                stripeData = obj.updateAcquisitionStatusWithNextTriggeringEnabled(stripeData,triggerTimes);
                if isempty(stripeData)
                    return
                end
            else
                stripeData = obj.updateAcquisitionStatus(stripeData,triggerTimes);
            end
            
            % fill in missing data in stripeData
            stripeData.epochAcqMode = obj.epochAcqMode;
            stripeData.acqNumber = obj.acqCounter + 1; % the current acquisition is always one ahead of the acquisition counter
            stripeData.frameNumberAcqMode = obj.frameCounter + 1; % the current frame number is always one ahead of the acquisition counter
            stripeData.endOfAcquisitionMode = stripeData.endOfAcquisition && stripeData.acqNumber == obj.trigAcqNumRepeats && obj.trigAcqNumRepeats > 0;
            
            % update counters
            if stripeData.endOfFrame
                obj.frameCounter = obj.frameCounter + 1;
            end
            
            if stripeData.endOfAcquisition
                obj.acqCounter = obj.acqCounter + 1;
            end
            
            stripeData.stripesRemaining = 0;
            
            % log stripe data
%             try
                obj.hLinScanLog.logStripe(stripeData);
%             catch ME
%                 most.idioms.reportError(ME);
%             end
            
            % remove 'logging-only' channels from stripeData
%             if obj.logEnable
%                 loggingOnlyChannels = setdiff(obj.hChannels.channelSave,obj.hChannels.channelDisplay);
% 
%                 for i = 1:numel(loggingOnlyChannels)
%                     chanNum = loggingOnlyChannels(i);
%                     stripeIdx = find(stripeData.channelNumbers==chanNum,1,'first');
% 
%                     stripeData.channelNumbers(stripeIdx) = [];
%                     stripeData.stripeData(stripeIdx) = [];
%                 end
%             end
            
%             % after logging, we can transpose the stripe data
%             for i = 1:numel(stripeData.stripeData)
%                 stripeData.stripeData{i} = stripeData.stripeData{i}';
%             end

            stripeData.stripeData = {};
            
            % publish stripe data
            obj.lastAcquiredFrame = stripeData;
            
            % control acquisition state
            if stripeData.endOfAcquisition
                if ~obj.trigNextStopEnable || stripeData.endOfAcquisitionMode
                    obj.zzAcquisitionDone;
                end
            end
            
            % done processing signal 'listeners' that data is ready to be read
            % limit display rate only if numStripes == 1, push all
            % stripeData.endOfAcquisition and all stripeData.frameNumberAcq == 1
            if obj.hAcq.acqParamBuffer.numStripes > 1 || stripeData.frameNumberAcq == 1 || stripeData.endOfAcquisition || toc(obj.lastDisplayTic) > 1/obj.mdfData.maxDisplayRate
                obj.lastDisplayTic = tic;
                % fprintf('Frame umber %d pushed to display\n',stripeData.frameNumberAcqMode);
                obj.stripeAcquiredCallback(obj,[]);
            else
                % fprintf('Frame Number %d not displayed\n',stripeData.frameNumberAcqMode);
            end
        end
        
        function stripeData = updateAcquisitionStatus(obj,stripeData,triggerTimes)
            if stripeData.endOfFrame
                if obj.frameCounter ~= 0 && stripeData.frameNumberAcq == 1
                    if ~isnan(triggerTimes.start)
                        % found a hardware timestamp!
                        obj.epochAcq = triggerTimes.start;
                    else
                        most.idioms.dispError('Warning: No timestamp for start trigger found. Estimating time stamp in software instead.\n');
                        obj.epochAcq = 86400 * ((now() - ((obj.hAcq.acqParamBuffer.frameTime - toc(startProcessingTime)) / 86400)) - obj.epochAcqMode); %the stripeAcquiredCallback happens _after_ the stripe is acquired, so subtract duration of frame. 86400 = seconds per day
                    end
                end

                stripeData.frameTimestamp = obj.epochAcq + ( stripeData.frameNumberAcq - 1 ) * obj.hAcq.acqParamBuffer.frameTime;
                
                if ~isnan(triggerTimes.stop) && triggerTimes.stop > obj.epochAcq
                    stripeData.endOfAcquisition = true;
                end
                
                if ~isnan(triggerTimes.next) && triggerTimes.next > obj.epochAcq
                    most.idioms.dispError('Next trigger detected, but acqusition is not configured to process it\n');
                end
            end
        end
        
        function stripeData = updateAcquisitionStatusWithNextTriggeringEnabled(obj,stripeData,triggerTimes)
            % if next triggering is enabed, a continuous acquisition is
            % used. this means that the stripeData 'end of acquisition'
            % flag has to be overwritten here
            stripeData.endOfAcquisition = false;
            
            persistent totalFrameCounter;
            persistent acquisitionActive;
            persistent currentAcq;
            persistent currentAcqFrame;
            persistent timeStamp;
            persistent startTriggerTimestamp;
            persistent nextTriggerTimestamp;
            persistent nextFileMarkerFlag;
            
            % initialize persistent variables
            if obj.frameCounter == 0 && stripeData.stripeNumber == 1
                acquisitionActive = true;
                totalFrameCounter = 0;
                currentAcq = 0;
                currentAcqFrame = 0;
                timeStamp = 0;
                startTriggerTimestamp = 0;
                nextTriggerTimestamp = 0;
                nextFileMarkerFlag = false;
            end
            
            if stripeData.endOfFrame
                totalFrameCounter = totalFrameCounter + 1;
                timeStamp = obj.hAcq.acqParamBuffer.frameTime * ( totalFrameCounter - 1 );
            end
            
            if ~acquisitionActive
                if ~isnan(triggerTimes.start) && obj.frameCounter > 0
                    acquisitionActive = true; %start Acquisition on next frame
                    startTriggerTimestamp = triggerTimes.start;
                end
                
                stripeData = [];
                return; %discard current stripe
            end
            
            stripeData.frameNumberAcq = currentAcqFrame + 1;
            
            if stripeData.endOfFrame
                currentAcqFrame = currentAcqFrame + 1;
                stripeData.frameTimestamp = timeStamp;
                
                if currentAcqFrame >= obj.framesPerAcq && obj.framesPerAcq > 0 && ~isinf(obj.framesPerAcq)
                    stripeData.endOfAcquisition = true;
                    acquisitionActive = false;
                    currentAcqFrame = 0;
                    currentAcq = currentAcq + 1;
                end
                
                if ~isnan(triggerTimes.stop)
                    stripeData.endOfAcquisition = true;
                    acquisitionActive = false;
                    currentAcqFrame = 0;
                    currentAcq = currentAcq + 1;
                end
                
                if ~isnan(triggerTimes.next)
                    nextFileMarkerFlag = true;
                    nextTriggerTimestamp = triggerTimes.next;
                    stripeData.nextFileMarkerTimestamp = triggerTimes.next;
                end
                
                if nextFileMarkerFlag && mod(obj.framesPerAcq,obj.framesPerStack) == 0
                    nextFileMarkerFlag = false;
                    stripeData.endOfAcquisition = true;
                    acquisitionActive = true;
                    currentAcqFrame = 0;
                    currentAcq = currentAcq + 1;
                end
                
                if stripeData.frameNumberAcq == 1
                    stripeData.acqStartTriggerTimestamp = startTriggerTimestamp;
                    stripeData.nextFileMarkerTimestamp = nextTriggerTimestamp;
                end
            end
        end

        function triggerTimes = processTriggers(obj,stripeData,startProcessingTime)
            triggerTimes = struct('start',NaN,'stop',NaN,'next',NaN);
            if ~stripeData.endOfFrame
                return
            end
            
            triggerTimesHardware = obj.hTrig.readTriggerTimes(); % returns a struct with fields start, stop, next
            
            % process start trigger
            if ~isnan(triggerTimesHardware.start)
                % hardware trigger takes precedence over software timestamp
                triggerTimes.start = triggerTimesHardware.start;
            elseif ~isnan(obj.trigStartSoftwareTimestamp)
                triggerTimes.start = 86400 * (obj.trigStartSoftwareTimestamp - obj.epochAcqMode);
            end
            
            % process stop trigger
            if ~obj.trigNextStopEnable
                triggerTimes.stop = NaN;
            elseif ~isnan(triggerTimesHardware.stop) && ~isempty(obj.trigStopInTerm)
                % hardware trigger takes precedence over software timestamp
                triggerTimes.stop = triggerTimesHardware.stop;
            elseif ~isnan(obj.trigStopSoftwareTimestamp)
                triggerTimes.stop = 86400 * (obj.trigStopSoftwareTimestamp - obj.epochAcqMode);
            end

            % process next trigger
            if ~obj.trigNextStopEnable
                triggerTimes.next = NaN;
            elseif ~isnan(triggerTimesHardware.next) && ~isempty(obj.trigNextInTerm)
                % hardware trigger takes precedence over software timestamp
                triggerTimes.next = triggerTimesHardware.next;
            elseif ~isnan(obj.trigNextSoftwareTimestamp)
                triggerTimes.next = 86400 * (obj.trigNextSoftwareTimestamp - obj.epochAcqMode);
            end

            % Reset trigger timestamps
            obj.trigStartSoftwareTimestamp = NaN;
            obj.trigStopSoftwareTimestamp  = NaN;
            obj.trigNextSoftwareTimestamp  = NaN;
        end
        
        function zzAcquisitionDone(obj)
            obj.haltAcquisition();
            
            if obj.trigAcqNumRepeats > 0 && obj.acqCounter >= obj.trigAcqNumRepeats;
                obj.abort(); % End of Acquisition Mode
            else
                if obj.trigAcqTypeExternal
                    obj.restartAcquisition();
                else
                    % do not start acquisition, instead wait for software trigger
                end
            end
        end
        
        function ziniConfigureRouting(obj)
            % Here it gets complicated
            if obj.hTrig.enabled
                % Auxiliary board enabled.
                if strcmp(obj.mdfData.deviceNameAcq,obj.mdfData.deviceNameGalvo) && ~strcmp(obj.mdfData.deviceNameGalvo,obj.mdfData.deviceNameAux)
                    % PMT inputs and XY Galvo output configured to be on
                    % the same board, but Aux board is separate
                    % Setup: the acqClock is generated on the auxiliary
                    % board and routed to the combined Acq/Galvo board
                    % the start trigger triggers the acqClock
                    obj.hAcq.hAI.sampClkSrc = obj.hTrig.sampleClockAcqTermInt;
                    obj.clockMaster = 'auxiliary';
                    obj.linkedSampleClkAcqCtl = true;
                elseif strcmp(obj.mdfData.deviceNameGalvo,obj.mdfData.deviceNameAux)
                    % The XY galvo output happens on the auxiliary board
                    obj.hAcq.hAI.sampClkSrc = obj.hTrig.sampleClockAcqTermInt;
                    obj.hCtl.hAO.sampClkSrc = 'OnboardClock';
                    
                    if ~isempty(obj.trigReferenceClkOutInternalTerm)
                        set(obj.hCtl.hAO,'sampClkTimebaseSrc',obj.trigReferenceClkOutInternalTerm);
                        set(obj.hCtl.hAO,'sampClkTimebaseRate',obj.trigReferenceClkOutInternalRate);
                    end
                    
                    obj.clockMaster = 'auxiliary';
                    obj.linkedSampleClkAcqCtl = false;
                else
                    error('Error initializing ''%s'' scanner.\nIf auxiliary board is defined, the XY Galvo output must be either configured to be on deviceNameAcq or deviceNameAux', obj.name);
                end                
            else
                % Auxiliary board disabled use only one board, no
                % beams/clock output, no synchronization with other boards
                if strcmp(obj.mdfData.deviceNameAcq,obj.mdfData.deviceNameGalvo)
                    % The XY galvo output happens on the auxiliary board
                    obj.clockMaster = 'controller';
                    obj.linkedSampleClkAcqCtl = false;
                else
                   error('Error initializing ''%s'' scanner.\nIf auxiliary board is not defined, deviceNameAcq and deviceNameGalvo must be equal', obj.name);
                end       
            end
        end
        
        function configureStartTrigger(obj) 
            if obj.trigAcqTypeExternal
                trigTerm = obj.trigAcqInTerm;
            else
                trigTerm = '';
            end
            
            switch obj.clockMaster
                case 'auxiliary'
                    if obj.linkedSampleClkAcqCtl
                        obj.hAcq.startTrigIn = '';
                        obj.hCtl.startTrigIn = '';
                    else
                        obj.hCtl.startTrigIn = obj.hTrig.sampleClockAcqTermInt;
                        obj.hCtl.startTrigEdge = 'rising';
                    end
                    obj.hTrig.sampleClkAcqStartTrigEdge = obj.trigAcqEdge;
                    obj.hTrig.sampleClkAcqStartTrigIn = trigTerm;
                case 'controller'
                    obj.hAcq.startTrigIn = trigTerm;
                    obj.hAcq.startTrigEdge = obj.trigAcqEdge;
                    obj.hCtl.startTrigIn = trigTerm;
                    obj.hCtl.startTrigEdge = obj.trigAcqEdge;
                    obj.hTrig.sampleClkAcqStartTrigIn = '';
                otherwise
                    assert(false);
            end
        end
        
        function tf = daqsInPxi(obj)
            daqs = unique({obj.mdfData.deviceNameAux obj.mdfData.deviceNameAcq obj.mdfData.deviceNameGalvo});
            hDevs = cellfun(@(x)dabs.ni.daqmx.Device(x),daqs,'UniformOutput',false);
            busTypes = cellfun(@(x)get(x,'busType'),hDevs,'UniformOutput',false);
            if all(strncmp(busTypes,'DAQmx_Val_PXI',13))
                chassisNums = cellfun(@(x)get(x,'PXIChassisNum'),hDevs);
                tf = all(chassisNums == chassisNums(1));
            else
                tf = false;
            end
       end
    end
    
    %% Internal API
    
    %%% PROPERTY ACCESS METHODS
    methods
        function set.channelOffsets(obj,val)
            assert(numel(val) == obj.channelsAvailable, 'Number of elements must match number of physical channels.');
            obj.channelOffsets = val;
            if obj.active
                obj.hAcq.updateBufferedOffsets();
            end
        end
        
        function set.linePhaseStep(obj,val)
            obj.mdlDummySetProp(val,'linePhaseStep');
        end
        
        function val = get.linePhaseStep(obj)
           val = 1 / obj.sampleRate;
        end
        
        function set.sampleRate(obj,val)
            val = obj.validatePropArg('sampleRate',val);
            assert(val <= obj.maxSampleRate,'Sample rate must be smaller or equal to %f Hz.',obj.maxSampleRate);
            if obj.componentUpdateProperty('sampleRate',val)
                % set sample rate in acquisition subsystem
                obj.hAcq.hAI.sampClkRate = val;
                % read sample rate back to get coerced value
                newVal = obj.hAcq.hAI.sampClkRate;
                obj.valCoercedWarning('sampleRate', val, newVal);
                
                % set property
                obj.sampleRate = val;
                
                % side effects
                obj.hTrig.sampleClkAcqFreq = obj.sampleRate;
                obj.sampleRateCtl = []; %updates the XY Galvo AO sample rate
                obj.linePhase = obj.linePhase;
            end
        end
        
        function set.sampleRateCtl(obj,~)
            % val is ignored this setter is just to update the AO output rate            
            maxOutputRate = min([obj.maxSampleRate,obj.maxSampleRateCtl,obj.MAX_REQUESTED_CTL_RATE]);
            
            % side effects
            % set AO output rate and read back to ensure it is set correctly
            if obj.linkedSampleClkAcqCtl
                if maxOutputRate >= obj.sampleRate
                    obj.sampleRateCtl = maxOutputRate;
                    set(obj.hCtl.hAO,'sampClkSrc','ai/SampleClock');
                    obj.hCtl.genSampClk = false;
                else
                    obj.sampleRateCtl = findNextPower2SampleRate(obj.sampleRate/4,maxOutputRate);
                    set(obj.hCtl.hAOSampClk.channels(1),'ctrTimebaseRate',obj.sampleRate);
                    set(obj.hCtl.hAOSampClk.channels(1),'pulseFreq',obj.sampleRateCtl);
                    set(obj.hCtl.hAO,'sampClkSrc','Ctr1InternalOutput');
                    obj.hCtl.genSampClk = true;
                end
            else
                obj.sampleRateCtl = findNextPower2SampleRate(obj.sampleRate,maxOutputRate);
            end
            
            obj.hCtl.hAO.sampClkRate = obj.sampleRateCtl;
            
            % check the actual output rate by reading it back from the task
            actualOutputRate = obj.hCtl.hAO.sampClkRate;
            assert(obj.sampleRateCtl == actualOutputRate,...
                ['Error: Output Rate for XY Galvo Control task could not be ',...
                 'set to requested value. Analog inputs and analog outputs ',...
                 'are out of sync']);
        end
        
        function set.pixelBinFactor(obj,val)
            val = obj.validatePropArg('pixelBinFactor',val);
            if obj.componentUpdateProperty('pixelBinFactor',val)
               obj.pixelBinFactor = val; 
            end
        end
        
        function set.logFilePerChannel(obj,val)
            if obj.componentUpdateProperty('logFilePerChannel',val)
                val = obj.validatePropArg('logFilePerChannel',val);
                
                obj.logFilePerChannel = val;
            end
        end
        
        function sz = get.defaultRoiFovSize(obj)
            rgs = obj.angularRange;
            sz = min(rgs) ./ rgs;
        end
        
        function rg = get.angularRange(obj)
            rg = [obj.mdfData.xGalvoAngularRange obj.mdfData.yGalvoAngularRange];
        end
    end
    
    %% ABSTRACT METHOD IMPLEMENTATIONS (scanimage.components.Scan2D)
    methods (Access = protected, Hidden)
        function val = accessScannersetPostGet(obj,~)
            pixelTime =obj.pixelBinFactor/obj.sampleRate;
            
            if obj.hSI.hStackManager.isFastZ && strcmp(obj.hSI.hFastZ.waveformType, 'step')
                flybackTime = max(obj.flybackTimePerFrame, obj.hSI.hFastZ.settlingTime);
            else
                flybackTime = obj.flybackTimePerFrame;
            end
            
            assert(~isempty(obj.mdfData.yGalvoAngularRange),'yGalvoAngularRange is not defined in machine data file');
            slow = scanimage.mroi.scanners.Galvo(...
                obj.mdfData.yGalvoAngularRange,...
                obj.mdfData.scanOffsetAngleY,...
                obj.mdfData.voltsPerOpticalDegreeY,...
                obj.mdfData.opticalDegreesPerMicronXY,...
                obj.flytoTimePerScanfield,...
                flybackTime,...
                obj.mdfData.scanParkAngleY,...
                obj.sampleRateCtl);
            
            assert(~isempty(obj.mdfData.xGalvoAngularRange),'xGalvoAngularRange is not defined in machine data file');
            fast = scanimage.mroi.scanners.Galvo(...
                obj.mdfData.xGalvoAngularRange,...
                obj.mdfData.scanOffsetAngleX,...
                obj.mdfData.voltsPerOpticalDegreeX,...
                obj.mdfData.opticalDegreesPerMicronXY,...
                obj.flytoTimePerScanfield,...
                obj.flybackTimePerFrame,...
                obj.mdfData.scanParkAngleX,...
                obj.sampleRateCtl);
             
            assert(~isempty(obj.mdfData.zPiezoRange),'zPiezoRange is not defined in machine data file');
            piezo = scanimage.mroi.scanners.Galvo(...
                obj.mdfData.zPiezoRange,...
                obj.mdfData.scanOffsetZ,...
                obj.mdfData.dummyValueZ,...
                obj.mdfData.voltsPerMicronZ,...
                obj.flytoTimePerScanfield,...
                obj.flybackTimePerFrame,...
                obj.mdfData.scanParkAngleZ,...
                obj.sampleRateCtl);
            
            hBeams=obj.hSI.hBeams;
            if hBeams.numInstances && ~isempty(obj.mdfData.beamDaqID)
                beamsSampleRate = findNextPower2SampleRate(obj.sampleRateCtl,hBeams.maxSampleRate(obj.mdfData.beamDaqID));
                beams = scanimage.mroi.scanners.Beams(hBeams.daqBeamIDs{obj.mdfData.beamDaqID},...
                    beamsSampleRate,...
                    hBeams.powers,...
                    hBeams.powerLimits,...
                    hBeams.flybackBlanking,...
                    hBeams.pzAdjust,...
                    hBeams.acqLengthConstants',...
                    hBeams.interlaceDecimation,...
                    hBeams.interlaceOffset,...
                    @(id,pf)hBeams.zprpBeamsPowerFractionToVoltage(id,pf),...
                    obj.linePhase,...
                    obj.beamClockDelay,...
                    obj.beamClockExtend,...
                    hBeams.powerBoxes);
            else
                beams = [];
            end
            % Create galvo galvo scannerset using hardware descriptions above
            val = scanimage.mroi.scannerset.GalvoGalvo(obj.name,fast,slow,piezo,beams,...
                obj.fillFractionSpatial,pixelTime,obj.bidirectional,true,obj.settleTimeFraction);
        end
        
        function accessBidirectionalPostSet(~,~)
            % Nothing to do here
        end
        
        function val = accessBidirectionalPreSet(~,val)
            % Nothing to do here
        end
        
        function val = accessChannelsFilterPostGet(~,~)
            val = 'None';
        end
        
        function val = accessChannelsFilterPreSet(obj,val)
            obj.errorPropertyUnSupported('channelsFilter',val);
            val = 'None';
        end
        
        function valActual = accessLinePhasePreSet(obj,val)
            valActual = obj.zprvRoundTimeToNearestSampleAcq(val);
        end        
        
        function val = accessLinePhasePostSet(obj,val)
            try
                if obj.active
                    obj.hAcq.updateBufferedPhaseSamples();
                    % regenerate beams output
                    obj.hSI.hBeams.updateBeamBufferAsync(true);
                end
            catch ME
                most.idioms.reportError(ME);
            end
        end
        
        function val = accessLinePhasePostGet(obj,val)
            %No-op
        end
        
        function val = accessBeamClockDelayPreSet(~,val)
            %for linscan beamClockDelay is in ticks. Must be an integer
            val = round(val);
        end

        function accessBeamClockDelayPostSet(obj, val)
            if obj.active
                obj.hSI.hBeams.updateBeamBufferAsync(true);
            end
        end
        
        function val = accessBeamClockExtendPreSet(~,val)            
            %for linscan beamClockDelay is in ticks. Must be an integer
            val = round(val);
        end
        
        function accessBeamClockExtendPostSet(obj,val)
            if obj.mdlInitialized
                most.idioms.warn('Not yet supported in LinScan');
            end
        end
        
        function accessChannelsAcquirePostSet(obj,val)
            obj.hSI.hBeams.powers = obj.hSI.hBeams.powers; % regenerate beams output
        end
        
        function val = accessFillFractionSpatialPreSet(~,val)
            % No-op
        end
        
        function accessFillFractionSpatialPostSet(~,~)
            % No-op
        end
		
	    function val = accessSettleTimeFractionPostSet(~,val)
            % No-op
        end
        
        function val = accessFlytoTimePerScanfieldPostGet(obj,val)
            val = obj.zprvRoundTimeToNearestSampleCtl(val);
        end
        
        function val = accessFlybackTimePerFramePostGet(obj,val)
            val = obj.zprvRoundTimeToNearestSampleCtl(val);
        end
        
        function val = accessLogAverageFactorPostSet(obj,val)
            %TODO: Implement this (if needed)
        end
        
        function accessLogFileCounterPostSet(obj,val)
            %TODO: Implement this (if needed)
        end
            
        function accessLogFilePathPostSet(obj,val)
            %TODO: Implement this (if needed)
        end
        
        function accessLogFileStemPostSet(obj,val)
            %TODO: Implement this (if needed)
        end
        
        function val = accessLogFramesPerFilePostSet(obj,val)
            %TODO: Implement this (if needed)
        end
        
        function val = accessLogFramesPerFileLockPostSet(obj,val)
            %TODO: Implement this (if needed)
        end
        
        function accessLogHeaderStringPostSet(obj,val)
            %TODO: Implement this (if needed)
        end
        
        function val = accessLogNumSlicesPreSet(obj,val)
            % TODO: Implement this
        end
        
        function val = accessTrigFrameClkOutInternalTermPostGet(obj,val)
            if obj.hTrig.enabled
                val = obj.hTrig.frameClockTermInt;
            else
                val = ''; % no trigger routing available without auxiliary port
            end
        end

        function val = accessTrigBeamClkOutInternalTermPostGet(obj,val)
            if obj.hTrig.enabled
                val = ''; % currently not implemented
            else
                val = ''; % no trigger routing available without auxiliary port
            end
        end
        
        function val = accessTrigAcqOutInternalTermPostGet(obj,val)
            val = ''; %Not supported in LinScan
        end
        
        function val = accessTrigReferenceClkOutInternalTermPostGet(obj,val)
            if obj.hTrig.enabled
                val = obj.hTrig.referenceClockTermInt;
            else
                val = '';
            end
        end
        
        function val = accessTrigReferenceClkOutInternalRatePostGet(obj,val)
            if obj.hTrig.enabled
                val = obj.hTrig.referenceClockRateInt;
            else
                val = [];
            end
        end        
        
        function val = accessTrigReferenceClkInInternalTermPostGet(obj,val)
            if isempty(obj.mdfData.deviceNameAux)
                val = '';
            else
                [device,terminal,frequency] = gethTrigRefClk(obj);
                val = terminal;
            end
        end
        
        function val = accessTrigReferenceClkInInternalRatePostGet(obj,val)
            if isempty(obj.mdfData.deviceNameAux)
                val = [];
            else
                [device,terminal,frequency] = gethTrigRefClk(obj);
                val = frequency;
            end
        end
		
        function val = accessTrigAcqInTermAllowedPostGet(obj,val)
             val =  {'','PFI0'};
        end
        
        function val = accessTrigNextInTermAllowedPostGet(obj,val)
            if obj.hTrig.enabled
                val = {'' , obj.hTrig.TRIG_LINE_NEXT};
            else
                val = {''}; % Next/Stop Triggering is not supported without an auxiliary board
            end
        end
        
        function val = accessTrigStopInTermAllowedPostGet(obj,val)
            if obj.hTrig.enabled
                val = {'' , obj.hTrig.TRIG_LINE_STOP};
            else
                val = {''}; % Next/Stop Triggering is not supported without an auxiliary board
            end
        end
        
        function  val = accessTrigAcqEdgePreSet(~,val)
            % Nothing to do here
        end
        
        function accessTrigAcqEdgePostSet(obj,val)
            obj.configureStartTrigger()
        end
        
        function val = accessTrigAcqInTermPreSet(~,val)
            % Nothing to do here
        end
        
        function accessTrigAcqInTermPostSet(obj,val)
            if isempty(obj.trigAcqInTerm)
                obj.trigAcqTypeExternal = false;
            end
            obj.configureStartTrigger();
        end
        
        function val = accessTrigAcqTypeExternalPreSet(~,val)
            % Nothing to do here
        end
        
        function accessTrigAcqTypeExternalPostSet(obj,val)
            obj.configureStartTrigger();
        end
        
        function val = accessTrigNextEdgePreSet(~,val)
            % Nothing to do here
        end
        
        function val = accessTrigNextInTermPreSet(obj,val)
            if ~isempty(val) && ~obj.hTrig.enabled
                val = '';
                warning('Cannot configure next trigger without an auxiliary DAQ board');
            end
        end
        
        function val = accessTrigNextStopEnablePreSet(obj,val)
            if val && ~obj.hTrig.enabled
                val = false;
                warning('Next/Stop triggering unavailable: no auxiliary board specified');
            end
        end
        
        function val = accessTrigStopEdgePreSet(~,val)
            % Nothing to do here
        end
        
        function val = accessFunctionTrigStopInTermPreSet(obj,val)
            if ~isempty(val) && ~obj.hTrig.enabled
                val = '';
                warning('Cannot configure stop trigger without an auxiliary DAQ board');
            end
        end
        
        function val = accessMaxSampleRatePostGet(obj,~)
            val = obj.hAcq.hAI.get('sampClkMaxRate');
        end
        
        function accessScannerFrequencyPostSet(obj,val)
            obj.errorPropertyUnSupported('scannerFrequency',val);
        end
        
        function val = accessScannerFrequencyPostGet(~,~)
            val = NaN;
        end
        
        function val = accessChannelsInputRangesPreSet(obj,val)
            hAI = obj.hAcq.hAI;

            % set the new inputRanges
            for i = 1:numel(val)
                inputRange = val{i};
                validateattributes(inputRange,{'numeric'},{'vector','numel',2});
                                
                hAIchan = hAI.channels(i);
                set(hAIchan,'min',inputRange(1));
                set(hAIchan,'max',inputRange(2));
            end
        end
        
        function val = accessChannelsInputRangesPostGet(obj,~)
            hAI = obj.hAcq.hAI; 
            
            numChans = numel(hAI.channels);
            ranges = cell(1,numChans);
            for i = 1:numChans
                hAIChan = hAI.channels(i);
                min = get(hAIChan,'min');
                max = get(hAIChan,'max');
                ranges{i} = [min, max];
            end
            val = ranges;
        end
        
        function val = accessChannelsAvailablePostGet(obj,~)
            if isempty(obj.channelsAvailable_)
                hDaqDeviceAcq = dabs.ni.daqmx.Device(obj.mdfData.deviceNameAcq);
                simultaneousSampling = get(hDaqDeviceAcq,'AISimultaneousSamplingSupported');
                if simultaneousSampling
                    channelNames = hDaqDeviceAcq.get('AIPhysicalChans');
                    channelNames = strsplit(channelNames, ', ');
                    val = length(channelNames);
                    val = min([val obj.MAX_NUM_CHANNELS numel(obj.mdfData.channelIDs)]);
                else
                    val = 1; % on multiplexed boards only one channel is available
                end
                
                obj.channelsAvailable_ = val;
            else
                val = obj.channelsAvailable_;
            end
        end
        
        function val = accessChannelsAvailableInputRangesPostGet(obj,~)
            %Retrieve AI voltage ranges directly - DAQmx interface does not currently 'get' vector-valued numeric properties correctly
            maxArrayLength = 100;
            [~, voltageRangeArray] = obj.hAcq.hAI.apiCall('DAQmxGetDevAIVoltageRngs', obj.mdfData.deviceNameAcq, zeros(maxArrayLength,1), maxArrayLength);
            
            voltageRangeArray(voltageRangeArray == 0) = []; %Remove trailing zeros
            val = reshape(voltageRangeArray,2,[])';
            val = mat2cell(val,ones(1,size(val,1)))'; % repack each line of the 2D array into a cell array
        end

        function val = accessScanPixelTimeMeanPostGet(obj,~)
            val = obj.pixelBinFactor / obj.sampleRate;
        end
        
        function val = accessScanPixelTimeMaxMinRatioPostGet(~,~)
            val = 1;
        end
        
        function val = accessChannelsAdcResolutionPostGet(obj,~)
            % assume all channels on the DAQ board have the same resolution
            channel = obj.hAcq.hAI.channels(1);
            val = get(channel,'resolution');
        end
        
        function val = accessChannelsDataTypePostGet(obj,~)
            if isempty(obj.channelsDataType_)
                singleSample = obj.hAcq.acquireSamples(1);
                val = class(singleSample);
                obj.channelsDataType_ = val;
            else
                val = obj.channelsDataType_;
            end
        end        
    end
    
    %% ABSTRACT HELPER METHOD IMPLEMENTATIONS (scanimage.components.Scan2D)
    methods (Access = protected)
        function fillFracTemp = fillFracSpatToTemp(~,fillFracSpat)
            fillFracTemp = fillFracSpat;
        end
        
        function fillFracSpat = fillFracTempToSpat(~,fillFracTemp)
            fillFracSpat = fillFracTemp;
        end
    end
    
    %% Helper functions
    methods (Access = protected)
        function actualTime = zprvRoundTimeToNearestSampleCtl(obj,time)
            samples = time * obj.sampleRateCtl; %#ok<*MCSUP>
            actualTime = round(samples) / obj.sampleRateCtl;
        end
        
        function actualTime = zprvRoundTimeToNearestSampleAcq(obj,time)
            samples = time * obj.sampleRate; %#ok<*MCSUP>
            actualTime = round(samples) / obj.sampleRate;
        end
        
        function generateTrigger(~,deviceName,triggerLine)
            % generates a trigger on PFI line specified by triggerLine
            % usage: generateTrigger('Dev1','PFI11');
            
            digitalLine = scanimage.util.translateTriggerToPort(triggerLine);
            
            hTask = most.util.safeCreateTask('Trigger Generator');
            try
            hTask.createDOChan(deviceName,digitalLine);
            hTask.writeDigitalData([0;1;0],0.5,true);
            catch err
                hTask.clear();
                rethrow(err);
            end
            hTask.clear();
        end
	
	    function [device,terminal,frequency] = gethTrigRefClk(obj)
            hAuxDaq = dabs.ni.daqmx.Device(obj.mdfData.deviceNameAux);
            device = hAuxDaq.deviceName; % to get the capitalization right
            switch get(hAuxDaq,'busType')
                case {'DAQmx_Val_PXI','DAQmx_Val_PXIe'}
                    terminal = ['/' obj.mdfData.deviceNameAux '/PXI_Clk10'];
                    frequency = 10e6;
                    
                    if ~isempty(obj.mdfData.referenceClockIn)
                        most.idioms.warn(['LinScan: Potential trigger routing conflict detected: ', ...
                            'Device %s inherits its reference clock from the PXI chassis 10MHz clock ', ...
                            'but an external reference clock is configured in the MDF setting referenceClockIn = ''%s''',...
                            'Please set referenceClockIn = '''' and remove all incoming clocks from this pin'],...
                        obj.mdfData.deviceNameAux,obj.mdfData.referenceClockIn);
                    end
                otherwise
                    if isempty(obj.mdfData.referenceClockIn)
                        terminal = '';
                        frequency = [];
                    else
                        terminal = ['/' obj.mdfData.deviceNameAux '/' obj.mdfData.referenceClockIn];
                        frequency = 10e6;
                    end
            end
        end
    end
end

%% local functions
function sampleRate = findNextPower2SampleRate(sourceSampleRate,maxSampleRate)
if isempty(sourceSampleRate) || isempty(maxSampleRate)
    sampleRate = [];
else
    sampleRate = min(sourceSampleRate, sourceSampleRate / 2^ceil(log2(sourceSampleRate/maxSampleRate)));
end
end

function s = zlclAppendDependsOnPropAttributes(s)
s.scannerset.DependsOn = horzcat(s.scannerset.DependsOn,{'pixelBinFactor','fillFractionSpatial'});
end


%--------------------------------------------------------------------------%
% LinScan.m                                                                %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
