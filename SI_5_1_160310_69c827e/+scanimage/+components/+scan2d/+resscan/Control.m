classdef Control < scanimage.interfaces.Class    
    properties (Hidden, SetAccess = immutable)
        hScan;
    end
    
    properties (Hidden, SetAccess = private)
        xGalvoExists = false;                % true if x-galvo exists 
    end
    
    properties (Hidden, Constant)
        AO_RATE_LIMIT = 200e3;               % limit for analog output rate
    end
    
    %% Original props
    %SCANNINGGALVO
    properties
        galvoFlyBackPeriods = 1;             % the number of scanner periods to fly back the galvo. can only be updated while the scanner is idle
        zoomFactor = 1;
        fillFractionSpatial = 1;
        resonantScannerZoomOutput = true;
        
        frameClockIn = 'PFI1';               % String identifying the input terminal connected to the frame clock. Values are 'PFI0'..'PFI15' and 'PXI_Trig0'..'PXI_Trig7'

        %simulated mode
        simulated=false;
    end
    
    % Live Values - these properties can be updated during an active acquisitio    
    properties (Dependent)
        galvoParkVoltsX;        
        galvoParkVoltsY;
    end
    
    % Internal Parameters
    properties (SetAccess = private, Hidden)
        hDaqDevice;
        
        hAOTaskResonantScannerZoom;
        
        hAOTaskGalvo;
        hCtrMSeriesSampClk;
        useSamplClkHelperTask = false;
        
        hAOTaskGalvoPark;
        
        hTimerContinuousFreqMeasurement;
        
        acquisitionActive = false;
        rateAOSampClk;
        resScanBoxCtrlInitialized;
        
        resonantScannerLastUpdate = clock;
        resonantScannerLastWrittenValue;
        
        activeFlag = false;
        
        galvoBufferUpdatingAsyncNow = false;
        galvoBufferNeedsUpdateAsync = false;
        galvoBufferUpdatingAsyncRetries = 0;
        galvoBuffer = [];
    end
    
    %% Lifecycle
    methods
        function obj = Control(hScan,simulated)
            if nargin < 1 || isempty(hScan)
                hScan = [];
            end
            
            if nargin < 2 || isempty(simulated)
                obj.simulated=false;
            else
                obj.simulated=simulated;
            end
            
            obj.hScan = hScan;
            
            %Get property values from machineDataFile
            validateattributes(obj.hScan.mdfData.galvoDeviceName,{'char'},{'vector','nonempty'});
            
            if ~isempty(obj.hScan.mdfData.galvoAOChanIDX)
                obj.xGalvoExists = true;
                validateattributes(obj.hScan.mdfData.galvoAOChanIDX,{'numeric'},{'scalar','nonnegative'});
                validateattributes(obj.hScan.mdfData.galvoVoltsPerOpticalDegreeX,{'numeric'},{'scalar','finite'});
            end
            validateattributes(obj.hScan.mdfData.galvoAOChanIDY,{'numeric'},{'scalar','nonnegative','nonempty'});
            
            if ~isfield(obj.hScan.mdfData,'resonantZoomDeviceName') || isempty(obj.hScan.mdfData.resonantZoomDeviceName)
                obj.hScan.mdfData.resonantZoomDeviceName = obj.hScan.mdfData.galvoDeviceName;
            end
            validateattributes(obj.hScan.mdfData.resonantZoomAOChanID,{'numeric'},{'scalar','nonnegative','nonempty'});
            
            validateattributes(obj.hScan.mdfData.galvoVoltsPerOpticalDegreeY,{'numeric'},{'scalar','finite'});
            validateattributes(obj.hScan.mdfData.rScanVoltsPerOpticalDegree,{'numeric'},{'scalar','finite','positive'});
            
            validateattributes(obj.hScan.mdfData.resonantScannerSettleTime,{'numeric'},{'scalar','nonnegative','nonempty'});
            
            obj.hTimerContinuousFreqMeasurement = timer('Period',1,'BusyMode','drop','ExecutionMode','fixedSpacing','TimerFcn',@obj.liveFreqMeasCallback);
        end
        
        function delete(obj)
            try
                if obj.acquisitionActive
                    obj.stop();
                end

                if ~obj.simulated
                    most.idioms.safeDeleteObj(obj.hTimerContinuousFreqMeasurement);
                    
                    % deactivate resonant scanner (may still be on depending on setting)
                    obj.resonantScannerActivate(false);
                    
                    % clear DAQmx buffered Tasks
                    if most.idioms.isValidObj(obj.hCtrMSeriesSampClk)
                        obj.hCtrMSeriesSampClk.clear();
                    end
                    
                    obj.hAOTaskGalvo.clear();
                    
                    % force AO Outputs to 0 Volts
                    obj.hAOTaskGalvoPark.writeAnalogData(0);
                    obj.hAOTaskResonantScannerZoom.writeAnalogData(0);
                    
                    % clear unbuffered Tasks
                    obj.hAOTaskGalvoPark.clear();
                    obj.hAOTaskResonantScannerZoom.clear();
                end
            catch ME
                obj.hDaqDevice.reset(); % hard reset the device to clear all routes and delete all tasks
                rethrow(ME);
            end
        end
        
        function initialize(obj)
            obj.hDaqDevice = dabs.ni.daqmx.Device(obj.hScan.mdfData.galvoDeviceName);
            
            if (~obj.simulated)
                obj.initializeTasks();
            else
                obj.rateAOSampClk = 3.333e6;
                obj.resScanBoxCtrlInitialized = true;
            end
            
            obj.zoomFactor = obj.zoomFactor; %self initialize output
        end
        
    end
    
    %% Public Methods
    methods        
        function start(obj)
            assert(~obj.acquisitionActive,'Acquisition is already active');      
            if (~obj.simulated)
                % Reconfigure the Tasks for the selected acquisition Model
                obj.updateTaskCfg();
                % this pause needed for the Resonant Scanner to reach
                % its amplitude and send valid triggers
                obj.activeFlag = true;
                obj.resonantScannerWaitSettle();
                % during resonantScannerWaitSettle a user might have clicked
                % 'abort' - which in turn calls obj.abort and unreserves
                % obj.hAOTaskGalvo; catch this by checking obj.activeflag
                if ~obj.activeFlag
                     errorStruct.message = 'Soft error: ResScan was aborted before the resonant scanner could settle.';
                     errorStruct.identifier = '';
                     errorStruct.stack = struct('file',cell(0,1),'name',cell(0,1),'line',cell(0,1));  
                     error(errorStruct); % this needs to be an error, so that Scan2D will be aborted correctly
                end
                
                obj.hAOTaskGalvo.start();
                if obj.useSamplClkHelperTask
                    obj.hCtrMSeriesSampClk.start();
                end 
                
                obj.hScan.liveScannerFreq = [];
                obj.hScan.lastLiveScannerFreqMeasTime = [];
            end
            
            obj.acquisitionActive = true;  
        end
        
        function stop(obj,soft)
            if nargin < 2 || isempty(soft)
                soft = false;
            end
            
            if (~obj.simulated)
                if obj.useSamplClkHelperTask
                    obj.hCtrMSeriesSampClk.abort();
                    obj.hCtrMSeriesSampClk.control('DAQmx_Val_Task_Unreserve'); % to allow the galvo to be parked
                end
                
                obj.hAOTaskGalvo.stop();
                obj.hAOTaskGalvo.control('DAQmx_Val_Task_Unreserve'); % to allow the galvo to be parked
                
                obj.activeFlag = false;
            end
                        
            %Park scanner
            % parkGalvo() has to be called after acquisitionActive is set to
            % false, otherwise we run into an infinite loop
            obj.acquisitionActive = false;
            if (~obj.simulated)
                obj.parkGalvo();
            end
            
            if obj.hScan.keepResonantScannerOn || soft
                obj.resonantScannerActivate(true);
            else
                obj.resonantScannerActivate(false);
            end
            
            obj.galvoBufferUpdatingAsyncNow = false;
        end
        
        function resonantScannerFreq = calibrateResonantScannerFreq(obj,averageNumSamples)
            if nargin < 2 || isempty(averageNumSamples)
                averageNumSamples = 100;
            end
            
            if ~obj.simulated
                if ~logical(obj.hScan.hAcq.hFpga.AcqStatusPeriodClockSettled)
                    resonantScannerFreq = NaN;
                else
                    t = tic;
                    nRej = obj.hScan.hAcq.hFpga.AcqStatusPeriodClockRejectedPulses;
                    
                    resonantPeriods = zeros(averageNumSamples,1);
                    resonantPeriods(1) = double(obj.hScan.hAcq.hFpga.AcqStatusPeriodClockPeriod) / obj.hScan.sampleRate;
                    for idx = 2:averageNumSamples
                        most.idioms.pauseTight(resonantPeriods(idx-1)*1.1);
                        resonantPeriods(idx) = double(obj.hScan.hAcq.hFpga.AcqStatusPeriodClockPeriod) / obj.hScan.sampleRate;
                    end
                    
                    nRej = obj.hScan.hAcq.hFpga.AcqStatusPeriodClockRejectedPulses - nRej;
                    dt = toc(t);
                    rejRate = (double(nRej) / dt);
                    if rejRate > (obj.hScan.mdfData.nominalResScanFreq * .01)
                        most.idioms.warn('%d period clock pulses (%d per second) were ignored because they were out of tolerance. Period clock may be noisy.', nRej, floor(rejRate));
                    end
                    
                    meanp = mean(resonantPeriods);
                    stddev = std(resonantPeriods);
                    minp = meanp - 3 * stddev;
                    maxp = meanp + 3 * stddev;
                    resonantPeriodsNrm = resonantPeriods(resonantPeriods > minp);
                    resonantPeriodsNrm = resonantPeriodsNrm(resonantPeriodsNrm < maxp);
                    outliers = numel(find(resonantPeriods < minp)) + numel(find(resonantPeriods > maxp));
                    
                    resonantFrequencies = 1 ./ resonantPeriods;
                    checkMeasurements(resonantFrequencies, outliers);
                    resonantScannerFreq = 1/mean(resonantPeriodsNrm);
                end
            else
                resonantScannerFreq = obj.hScan.mdfData.nominalResScanFreq + rand(1);
            end
            
            % nested functions
            function checkMeasurements(measurements, outliers)
                maxResFreqStd = 10;
                maxResFreqError = 0.1;
                
                resFreqMean = mean(measurements);
                resFreqStd  = std(measurements);
                
                resFreqNom = obj.hScan.mdfData.nominalResScanFreq;
                
                if abs((resFreqMean-resFreqNom)/resFreqNom) > maxResFreqError
                    most.idioms.warn('The measured resonant frequency does not match the nominal frequency. Measured: %.1fHz Nominal: %.1fHz',...
                        resFreqMean,resFreqNom) ;
                end
                
                if outliers > 0
                    if outliers > 1
                        s = 's';
                    else
                        s = '';
                    end
                    msg = sprintf('%d outlier%s will be ignored in calculation.\n',outliers,s);
                else
                    msg = '';
                end
                
                if resFreqStd > maxResFreqStd
                    plotMeasurement(measurements);
                    most.idioms.dispError(['The resonant frequency is unstable. Mean: %.1fHz, SD: %.1fHz.\n',...
                               'Possible solutions:\n\t- Reduce the zoom\n\t- increase the value of resonantScannerSettleTime in the Machine Data File\n',...
                               '\t- set hSI.hScanner(''%s'').keepResonantScannerOn = true\n%s'],...
                               resFreqMean,resFreqStd,obj.hScan.name,msg);
                end
            end
           
            function plotMeasurement(measurements)
                persistent hFig
                if isempty(hFig) || ~ishghandle(hFig)
                    hFig = figure('Name','Resonant scanner frequency','NumberTitle','off','MenuBar','none');
                end
                
                clf(hFig);
                figure(hFig); %bring to front
                
                hAx = axes('Parent',hFig);
                plot(hAx,measurements);
                title(hAx,'Resonant scanner frequency');
                xlabel(hAx,'Measurements');
                ylabel(hAx,'Resonant Frequency [Hz]');
            end
        end
        
        function resonantScannerActivate(obj,activate,volts)
           if nargin < 2 || isempty(activate)
               activate = true;
           end
           
           if activate
               if nargin < 3 || isempty(volts)
                   resScanOutputPoint = obj.nextResonantVoltage;
               else
                   resScanOutputPoint = volts;
               end
           else
               resScanOutputPoint = 0;
           end
           
           obj.resonantScannerUpdateOutputVolts(resScanOutputPoint);
        end
        
        function resonantScannerWaitSettle(obj,settleTime)
            if nargin < 2 || isempty(settleTime)
            	timeToWait = obj.getRemainingResSettlingTime();
			else
            	timeToWait = obj.getRemainingResSettlingTime(settleTime);
            end
            
            if timeToWait > 0
                %fprintf('Waiting %f seconds for resonant scanner to settle\n',timeToWait);
                pause(timeToWait);
            end
        end
        
        function timeToWait = getRemainingResSettlingTime(obj,settleTime)
            if nargin < 2 || isempty(settleTime)
                settleTime = max(0.5,obj.hScan.mdfData.resonantScannerSettleTime);
            end
            
            timeSinceLastAOUpdate = etime(clock,obj.resonantScannerLastUpdate);
            timeToWait = max(0, settleTime-timeSinceLastAOUpdate);
        end
        
        function parkGalvo(obj)
           assert(~obj.acquisitionActive,'Cannot park galvo while scanner is active');           
           if obj.xGalvoExists
               parkVolts = [obj.galvoParkVoltsX obj.galvoParkVoltsY];
           else
               parkVolts = obj.galvoParkVoltsY;
           end

           obj.forceGalvoVolts(parkVolts);
        end
        
        function centerGalvo(obj)
           assert(~obj.acquisitionActive,'Cannot center galvo while scanner is active');           
           if obj.xGalvoExists
               parkVolts = [0 0];
           else
               parkVolts = 0;
           end

           obj.forceGalvoVolts(parkVolts);
        end
        
        function pointGalvoDeg(obj,val)
            volts = val * obj.hScan.mdfData.galvoVoltsPerOpticalDegreeY;
            obj.pointGalvoVolts(volts);
        end
        
        function pointGalvoVolts(obj,val)
           assert(~obj.acquisitionActive,'Cannot point galvo while scan is active');
           obj.forceGalvoVolts(val);
        end
        
        function pointResAmplitudeDeg(obj,angle)
            volts = obj.hScan.zzzResonantFov2Volts(angle/obj.hScan.mdfData.resonantAngularRange);
            obj.pointResAmplitudeVolts(volts);
        end
        
        function pointResAmplitudeVolts(obj,val)
            assert(~obj.acquisitionActive,'Cannot change resonant scanner amplitude while scan is active');
            obj.resonantScannerActivate(true,val);
        end
        
        function updateLiveValues(obj)
            if (~obj.simulated)
                if obj.acquisitionActive
                    try
                        obj.hScan.hSI.zzzUpdateAO();
                        obj.updateTaskCfg(true);
                    catch ME
                        % ignore DAQmx Error 200015 since it is irrelevant here
                        % Error message: "While writing to the buffer during a
                        % regeneration the actual data generated might have
                        % alternated between old data and new data."
                        if isempty(strfind(ME.message, '200015'))
                            rethrow(ME)
                        end
                    end
                else
                    if obj.hScan.keepResonantScannerOn
                        obj.resonantScannerActivate();
                    end
                    
                    % if the parking position for the Galvo was updated, apply
                    % the new settings.
                    obj.parkGalvo();
                end
            end
        end
    end
    
    %% Private Methods
    methods (Hidden)
        function v = nextResonantVoltage(obj)
            v = obj.hScan.scannerset.resonantScanVoltage(obj.hScan.currentRoiGroupScannerCoords);
        end
        
        function v = nextResonantFov(obj)
            v = obj.hScan.scannerset.resonantScanFov(obj.hScan.currentRoiGroupScannerCoords);
        end
    end
    
    methods (Access = private)
        function resonantScannerUpdateOutputVolts(obj,val)            
            if abs(val - obj.resonantScannerLastWrittenValue) > 0.0001
                obj.resonantScannerLastUpdate = clock;
            end
            
            if (~obj.simulated) && ~obj.hScan.disableResonantZoomOutput
                obj.hAOTaskResonantScannerZoom.writeAnalogData(val);
            end
            
            if val
                obj.hScan.flagZoomChanged = true;
                obj.hScan.linePhase = obj.hScan.zzzEstimateLinePhase(val);
            end
            
            obj.resonantScannerLastWrittenValue = val;
            notify(obj.hScan,'resonantScannerOutputVoltsUpdated');
        end
        
        function initializeTasks(obj) 
            try
            % create Tasks
            obj.hAOTaskGalvo = most.util.safeCreateTask([obj.hScan.name '-GalvoCtrlGalvoPosition']);
            obj.hAOTaskResonantScannerZoom = most.util.safeCreateTask([obj.hScan.name '-GalvoCtrlresonantScannerZoomVolts']);
            obj.hAOTaskGalvoPark = most.util.safeCreateTask([obj.hScan.name '-ParkGalvoCtrlAO']);
            
            %set up buffered AO Task to control the Galvo Scan
            if obj.xGalvoExists
                obj.hAOTaskGalvo.createAOVoltageChan(obj.hScan.mdfData.galvoDeviceName,obj.hScan.mdfData.galvoAOChanIDX,'X Galvo Control',-10,10);
            end
            obj.hAOTaskGalvo.createAOVoltageChan(obj.hScan.mdfData.galvoDeviceName,obj.hScan.mdfData.galvoAOChanIDY,'Y Galvo Control',-10,10);
            maxSampleRate = min(scanimage.util.daqTaskGetMaxSampleRate(obj.hAOTaskGalvo),obj.AO_RATE_LIMIT);
            
            switch obj.hDaqDevice.productCategory
                case 'DAQmx_Val_AOSeries'
                    most.idioms.warn('Support for PXIe-6738/6739 is experimental. Some features may not work.');
                    obj.hAOTaskGalvo.cfgSampClkTiming(maxSampleRate,'DAQmx_Val_FiniteSamps',2); % length of output will be overwritten later
                    obj.rateAOSampClk = get(obj.hAOTaskGalvo,'sampClkRate');
                    obj.hAOTaskGalvo.cfgDigEdgeStartTrig(obj.frameClockIn);
                    obj.hAOTaskGalvo.set('startTrigRetriggerable',1);
                    obj.useSamplClkHelperTask = false;
                case 'DAQmx_Val_XSeriesDAQ'
                    obj.hAOTaskGalvo.cfgSampClkTiming(maxSampleRate,'DAQmx_Val_FiniteSamps',2); % length of output will be overwritten later
                    obj.rateAOSampClk = get(obj.hAOTaskGalvo,'sampClkRate');
                    obj.hAOTaskGalvo.cfgDigEdgeStartTrig(obj.frameClockIn);
                    obj.hAOTaskGalvo.set('startTrigRetriggerable',1);
                    obj.useSamplClkHelperTask = false;
                case 'DAQmx_Val_MSeriesDAQ'
                    % the M series does not support native retriggering for
                    % AOs. Workaround: Use counter to produce sample clock
                    obj.hCtrMSeriesSampClk = most.util.safeCreateTask([obj.hScan.name '-M-Series helper task']);
                    obj.hCtrMSeriesSampClk.createCOPulseChanFreq(obj.hScan.mdfData.galvoDeviceName,0,[],maxSampleRate);
                    obj.rateAOSampClk = get(obj.hCtrMSeriesSampClk.channels(1),'pulseFreq');
                    obj.hCtrMSeriesSampClk.channels(1).set('pulseTerm',''); % we do not need to export the sample clock to a PFI. delete
                    obj.hCtrMSeriesSampClk.cfgImplicitTiming('DAQmx_Val_FiniteSamps',2); % length of output will be overwritten later
                    obj.hCtrMSeriesSampClk.cfgDigEdgeStartTrig(obj.frameClockIn);
                    obj.hCtrMSeriesSampClk.set('startTrigRetriggerable',1);
                    
                    % setup hAOTaskGalvo to use the sample clock generated by the counter
                    samplClkInternalOutputTerm = sprintf('/%sInternalOutput',obj.hCtrMSeriesSampClk.channels(1).chanNamePhysical);
                    obj.hAOTaskGalvo.cfgSampClkTiming(obj.rateAOSampClk,'DAQmx_Val_ContSamps',2,samplClkInternalOutputTerm);
                    obj.useSamplClkHelperTask = true;
                otherwise
                    error('Primary DAQ Device needs to be either M-series or X-series');
            end
            
            %set up unbuffered Task to move the Galvo to a given position
            if obj.xGalvoExists
                obj.hAOTaskGalvoPark.createAOVoltageChan(obj.hScan.mdfData.galvoDeviceName,obj.hScan.mdfData.galvoAOChanIDX,'X Galvo Control',-10,10);
            end
            obj.hAOTaskGalvoPark.createAOVoltageChan(obj.hScan.mdfData.galvoDeviceName,obj.hScan.mdfData.galvoAOChanIDY,'Y Galvo Control',-10,10);
            obj.parkGalvo();
            
            %set up unbuffered Task to set the resonant scanner zoom level
            obj.hAOTaskResonantScannerZoom.createAOVoltageChan(obj.hScan.mdfData.resonantZoomDeviceName,obj.hScan.mdfData.resonantZoomAOChanID,[],0,5);
            obj.resonantScannerActivate(false); % set output to zero       
            
            catch ME
                obj.hDaqDevice.reset(); %clear all routes
                delete(obj)
                rethrow(ME);
            end
            
            obj.resScanBoxCtrlInitialized = true;
        end
             
        function updateTaskCfg(obj, isLive)            
            if nargin < 2 || isempty(isLive)
                isLive = false;
            end
            
            recurse = false;
            
            [scanPoints,samplesPerFrame] = obj.getGalvoScanOutputPts();
            
            % Handle Resonant Scanner.
            % Update AO Buffers (Performance seems to be better when updating the galvo task last.
            resScanOutputPoint_ = scanPoints(1,1);
            obj.resonantScannerUpdateOutputVolts(resScanOutputPoint_);
            
            % Handle Galvo.
            if obj.xGalvoExists
                obj.galvoBuffer = scanPoints(:,2:3);
            else
                obj.galvoBuffer = scanPoints(:,2);
            end
            bufferLength = length(obj.galvoBuffer);
            assert(bufferLength > 0, 'AO generation error. Galvo control waveform length is zero.');
            
            % If acq is not live make sure buffered tasks are stopped
            if ~isLive
                if obj.useSamplClkHelperTask
                    obj.hCtrMSeriesSampClk.abort();
                end

                obj.hAOTaskGalvo.abort();
                obj.hAOTaskGalvoPark.control('DAQmx_Val_Task_Unreserve'); % to allow the galvo to be parked
                obj.hAOTaskGalvo.control('DAQmx_Val_Task_Unreserve'); % to allow the galvo to be parked
            
                oldSampleRate = obj.rateAOSampClk;
                
                if obj.useSamplClkHelperTask
                    obj.hCtrMSeriesSampClk.set('sampQuantSampPerChan',length(obj.galvoBuffer));
                    obj.hCtrMSeriesSampClk.cfgDigEdgeStartTrig(obj.frameClockIn,'DAQmx_Val_Rising');
                    obj.hCtrMSeriesSampClk.set('startTrigRetriggerable',true);
                    obj.hSampleClockAcq.channels(1).set('ctrTimebaseSrc',obj.hScan.trigReferenceClkOutInternalTerm);
                    obj.hSampleClockAcq.channels(1).set('ctrTimebaseRate',obj.hScan.trigReferenceClkOutInternalRate);
                    obj.hAOTaskGalvo.set('sampQuantSampPerChan',samplesPerFrame);
                    
                    obj.rateAOSampClk = get(obj.hCtrMSeriesSampClk.channels(1),'pulseFreq');
                else
                    obj.hAOTaskGalvo.cfgDigEdgeStartTrig(obj.frameClockIn,'DAQmx_Val_Rising');
                    obj.hAOTaskGalvo.set('startTrigRetriggerable',true);
                    obj.hAOTaskGalvo.set('sampQuantSampPerChan',samplesPerFrame);
                    if obj.hScan.useResonantTimebase
                        obj.hAOTaskGalvo.set('sampClkTimebaseSrc',obj.hScan.hTrig.getPXITerminal('beamModifiedLineClockOut'));
                        obj.hAOTaskGalvo.set('sampClkTimebaseRate',obj.hScan.resonantTimebaseNominalRate);
                    else
                        obj.hAOTaskGalvo.set('sampClkTimebaseSrc',obj.hScan.trigReferenceClkOutInternalTerm);
                        obj.hAOTaskGalvo.set('sampClkTimebaseRate',obj.hScan.trigReferenceClkOutInternalRate);
                    end
                    obj.rateAOSampClk = get(obj.hAOTaskGalvo,'sampClkRate');
                end
                % setting the sampClkTimebaseSrc might change the
                % rateAOSampClk. in this case execute updateTaskCfg one
                % more time
                if obj.rateAOSampClk ~= oldSampleRate
                        recurse = true;
                end
                
                timeout = 3;
            else
                timeout = nan;
            end
            
            % Update AO Buffers
            obj.hAOTaskGalvo.cfgOutputBuffer(bufferLength);
            obj.updateGalvoBufferAsync(timeout);
            
            if recurse
                obj.updateTaskCfg();
            end
        end
        
        function updateGalvoBufferAsync(obj, timeout)
            
            if nargin < 2 || isempty(timeout)
                timeout = nan;
            end
            
            if obj.galvoBufferUpdatingAsyncNow
                % async call currently in progress. schedule update after current update finishes
                obj.galvoBufferNeedsUpdateAsync = true;
            else
                obj.galvoBufferNeedsUpdateAsync = false;
                obj.galvoBufferUpdatingAsyncNow = true;
                obj.hAOTaskGalvo.writeAnalogDataAsync(obj.galvoBuffer,[],[],[],@(src,evt)obj.updateGalvoBufferAsyncCallback(src,evt));
            end
            
            if ~isnan(timeout)
                t = tic;
                while obj.galvoBufferUpdatingAsyncNow
                    pause(.01);
                    assert(toc(t) < timeout, 'Galvo buffer write timed out.');
                end
            end
        end
        
        function updateGalvoBufferAsyncCallback(obj,~,evt)
            obj.galvoBufferUpdatingAsyncNow = false;
            
            if evt.status ~= 0 && evt.status ~= 200015 && obj.hScan.active
                fprintf(2,'Error updating galvo buffer: %s\n%s\n',evt.errorString,evt.extendedErrorInfo);
                
                if obj.galvoBufferUpdatingAsyncRetries < 3 || obj.galvoBufferNeedsUpdateAsync
                    obj.galvoBufferUpdatingAsyncRetries = obj.galvoBufferUpdatingAsyncRetries + 1;
                    fprintf(2,'Scanimage will retry update...\n');
                    obj.updateGalvoBufferAsync();
                else
                    obj.galvoBufferUpdatingAsyncRetries = 0;
                end
            else
                obj.galvoBufferUpdatingAsyncRetries = 0;

                if obj.galvoBufferNeedsUpdateAsync
                    obj.updateGalvoBufferAsync();
                end
            end
        end
        
        function [dataPoints,samplesPerFrame] = getGalvoScanOutputPts(obj)
            
            dataPoints = [obj.hScan.hSI.scannerAO.ao_volts.R obj.hScan.hSI.scannerAO.ao_volts.G];
            samplesPerFrame = obj.hScan.hSI.scannerAO.ao_samplesPerTrigger.G;
            
            %Replace any NaN's with the maximum volts for the resonant scanner.
            %This handles any and all invalid values passed into the resonant scanner.
            maxResonantVolts = max(dataPoints(:,1));
            dataPoints(isnan(dataPoints),1) = maxResonantVolts;
            
            assert(~mod(length(dataPoints),samplesPerFrame),'Length of dataPoints has to be divisible by samplesPerFrame');
        end
        
        function forceGalvoVolts(obj,value)
            if obj.acquisitionActive
                obj.stop();
            end
            
            if (~obj.simulated)
                obj.hAOTaskGalvoPark.writeAnalogData(value);
            end
        end
        
        function liveFreqMeasCallback(obj,~,~)
            obj.hScan.liveScannerFreq = obj.calibrateResonantScannerFreq(1);
            obj.hScan.lastLiveScannerFreqMeasTime = clock;
        end
    end
    
    %% Property Set Methods
    methods        
        function set.frameClockIn(obj,value)
            assert(~obj.acquisitionActive,'Cannot change %s while scanner is active','frameClockIn');
            validateattributes(value,{'char'},{'vector','nonempty'});
            
            obj.frameClockIn = value;
            % settings are applied in updateTaskCfg()
        end
        
        function set.galvoFlyBackPeriods(obj,value)
            assert(~obj.acquisitionActive,'Cannot change %s while scanner is active','galvoFlyBackPeriods');
            assert(value >= 1,'galvoFlyBackPeriods must be greater or equal to 1');
            obj.galvoFlyBackPeriods = value;
        end
        
        function value = get.galvoParkVoltsX(obj)
            value = obj.hScan.mdfData.galvoParkDegreesX * obj.hScan.mdfData.galvoVoltsPerOpticalDegreeX;
        end
        
        function value = get.galvoParkVoltsY(obj)
            value = obj.hScan.mdfData.galvoParkDegreesY * obj.hScan.mdfData.galvoVoltsPerOpticalDegreeY;
        end
    end
end


%--------------------------------------------------------------------------%
% Control.m                                                                %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
