classdef ResScan < scanimage.components.Scan2D & most.HasMachineDataFile
    %% USER PROPS
    properties (SetObservable, Transient)
        linePhaseMode = 'Nearest Neighbor';   % Specifies method for estimating line phase if it is not measured at the current resonant amplitude
        % Note: This is all just guessing. The user must either explicitly
        % set scan phases for all zoom levels or we have to make a way for
        % the scanner to automatically set the scan phase for perfect bidi
        % alignment.
        %
        % Interpolate:      Linearly interpolate between next lower and next
        %                   higher zoom factor with a set scan phase.
        % Nearest Neighbor: Choose between scan phase of next lower and next
        %                   higher zoom factor with a set scan phase, whichever zoom factor is
        %                   closest to current.
        % Next Lower:       Choose the scan phase of the next lower zoom factor
        %                   with a set scan phase.
        % Next Higher:      Choose the scan phase of the next higher zoom factor
        %                   with a set scan phase.
        
        keepResonantScannerOn = false;  % Indicates that resonant scanner should always be on. Avoids settling time and temperature drift
        sampleRate;
        pixelBinFactor = NaN;
        channelOffsets;
    end
    
    %These are stored in class data file, so don't cfg
    properties (SetObservable, Transient, Dependent)
        resonantLimitedFovMode = true;
        useNonlinearResonantFov2VoltsCurve = false;
    end
    
    %% FRIEND PROPS
    properties (Hidden)
        enableContinuousFreqMeasurement = false;
        
        useResonantTimebase = false;
        resonantSettlingPeriods = 100;
        nomResPeriodTicks;
        resonantTimebaseNominalRate = 5e6;
        resonantTimebaseTicksPerPeriod;
        scannerPeriodRTB;
        
        useFpgaBiDir = false;
    end
    
    properties (Hidden, Dependent, GetAccess = ?scanimage.interfaces.Class)
        resonantScannerLastWrittenValue;
    end
    
    properties (Hidden, SetAccess = ?scanimage.interfaces.Class)
        disableResonantZoomOutput = false;
        flagZoomChanged = false;        % (Logical) true if user changed the zoom via spinner controls.
        
        liveScannerFreq;
        lastLiveScannerFreqMeasTime;
    end
    
    properties (Hidden, SetAccess = ?scanimage.interfaces.Class, Dependent)
        fpgaShutterOut;     % true/false sets the digital signal on the fpgaShutterOutTerm to close/open the shutter
        fpgaShutterOutTerm; % set output terminal for shutter on FPGA
    end
 
    %% INTERNAL PROPS
    properties (Hidden, SetAccess = private)            
        hAcq;                               % handle to image acquisition system
        hCtl;                               % handle to galvo control system
        hTrig;                              % handle to trigger system
        
        hCalFig = [];
        hCalPlot = [];
        hCalPlotPt = [];
        
        firstInit = true;
    end
    properties (Hidden, SetAccess = private)
        resonantLimitedFovMode_ = true;
        useNonlinearResonantFov2VoltsCurve_ = false;
    end
    
    properties (Hidden, SetAccess = protected, Dependent) 
        %         trigAcqInTermAllowed;               % cell array of strings with allowed terminal names for the acq trigger (e.g. {'PFI1','PFI2'})
        %         trigNextInTermAllowed;              % cell array of strings with allowed terminal names for the acq trigger (e.g. {'PFI1','PFI2'})
        %         trigStopInTermAllowed;              % cell array of strings with allowed terminal names for the acq trigger (e.g. {'PFI1','PFI2'})        
        linePhaseStep;                      % [s] minimum step size of the linephase
        multiChannel;                       % true if numel(channelsActive)> 1; used to configure the DMA Fifo configuration on the FPGA        
        
        periodsPerFrame;
        digitalIODeviceType;
        digitalIODaqName;
    end
    
    properties (Hidden, SetAccess = protected)
        %allowedTriggerInputTerminals;
        %allowedTriggerInputTerminalsMap;
        
        linePhase_;                     % Transient linePhase value set at current zoom; if non-empty will be stored to linePhaseMap on change of zoom or abort
        linePhaseMap;                   % containers.Map() that holds the LUT values for scan phase. Saved on acq abort to class data file.
        scanFreqMap;                    % containers.Map() that holds the LUT values for scan frequency. Saved on acq abort to class data file.
        resFov2VoltsMap;                % containers.Map() that holds the LUT values for scan angle to voltage conversion to correct for non linearity
        defaultRoiFovSize;
        angularRange;
        supportsRoiRotation = false;
    end
    
    %%% Abstract prop realizations (most.Model)
    properties (Hidden, SetAccess=protected)
        mdlPropAttributes = zlclAppendDependsOnPropAttributes(scanimage.components.Scan2D.scan2DPropAttributes());
        mdlHeaderExcludeProps = {'logFileStem' 'logFilePath' 'logFileCounter'};
    end    
        
    %%% Abstract property realizations (most.HasMachineDataFile)
    properties (Constant, Hidden)
        %Value-Required properties
        mdfClassName = mfilename('class');
        mdfHeading = 'ResScan';
        
        %Value-Optional properties
        mdfDependsOnClasses; %#ok<MCCPI>
        mdfDirectProp; %#ok<MCCPI>
        mdfPropPrefix; %#ok<MCCPI>
        
        mdfOptionalVars = struct(...
            'enableRefClkOutput', false, ...      % Enables/disables the 10MHz reference clock output on PFI14 of the digitalIODevice
            ...
            'PeriodClockDebounceTime', 100e-9, ...% [s] time the period clock has to be stable before a change is registered
            'TriggerDebounceTime', 500e-9, ...    % [s] time acquisition, stop and next trigger to be stable before a change is registered
            ...
            ... % auxTriggersEnable, I2CEnable and photonCountingEnable are mutually exclusive
            'auxTriggersEnable', true, ...
            'auxTriggersTimeDebounce', 1e-6, ...       % [s] time an aux trigger needs to be high for registering an edge (seconds)
            'auxTriggerLinesInvert', false(4,1), ...   % [logical] 1x4 vector specifying polarity of aux trigger inputs
            ...
            'photonCountingEnable', false, ...
            'photonCountingDisableAveraging', false, ... % disable averaging of samples into pixels; instead accumulate samples
            'photonCountingScaleByPowerOfTwo', 8, ...    % for use with photonCountingDisableAveraging == false; scale count by 2^n before averaging to avoid loss of precision by integer division
            'photonCountingDebounce', 25e-9, ...         % [s] time the TTL input needs to be stable high before a pulse is registered
            ...
            'I2CEnable', false, ...
            'I2CAddress', uint8(0), ...   		 % [byte] I2C address of the FPGA
            'I2CDebounce', 500e-9, ...   		 % [s] time the I2C signal has to be stable high before a change is registered
            'I2CStoreAsChar', false, ... 	     % if false, the I2C packet bytes are stored as a uint8 array. if true, the I2C packet bytes are stored as a string. Note: a Null byte in the packet terminates the string
            'I2CDisableAckOutput', false ...     % the FPGA confirms each packet with an ACK bit by actively pulling down the SDA line. I2C_DISABLE_ACK_OUTPUT = true disables the FPGA output
            );
    end
    
    %%% Abstract prop realization (scanimage.interfaces.Component)
    properties (SetAccess = protected, Hidden)
        numInstances = 0;
    end        
    
    %%% Abstract property realizations (scanimage.subystems.Scan2D)
    properties (Constant)
        scannerType = 'Resonant';
    end
    
    %%% Constants
    properties (Constant, Hidden)
        MAX_NUM_CHANNELS = 4;               % Maximum number of channels supported
        
        COMPONENT_NAME = 'ResScan';                                                     % [char array] short name describing functionality of component e.g. 'Beams' or 'FastZ'
        PROP_TRUE_LIVE_UPDATE = {...              % Cell array of strings specifying properties that can be set while the component is active
            'linePhase','beamClockDelay','logFileCounter','useNonlinearResonantFov2VoltsCurve'};
        PROP_FOCUS_TRUE_LIVE_UPDATE = {};    % Cell array of strings specifying properties that can be set while focusing
        DENY_PROP_LIVE_UPDATE = {'framesPerAcq','trigAcqTypeExternal',...  % Cell array of strings specifying properties for which a live update is denied (during acqState = Focus)
            'trigAcqTypeExternal','trigNextStopEnable','trigAcqInTerm',...
            'trigNextInTerm','trigStopInTerm','trigAcqEdge','trigNextEdge',...
            'trigStopEdge','stripeAcquiredCallback'...
            'logAverageFactor','logFilePath',...
            'logFileStem','logFramesPerFile','logFramesPerFileLock',...
            'logHeaderString','logNumSlices'};
        
        FUNC_TRUE_LIVE_EXECUTION = {'readStripeData','trigIssueSoftwareAcq','measureScannerFrequency',...
            'trigIssueSoftwareNext','trigIssueSoftwareStop',...
            'plotFov2VoltageCurve','adjustResVoltageCal','clearResFov2VoltsCal',...
            'createResFov2VoltsCalControlPoint','removeResFov2VoltsCalControlPoint'};  % Cell array of strings specifying functions that can be executed while the component is active
        FUNC_FOCUS_TRUE_LIVE_EXECUTION = {}; % Cell array of strings specifying functions that can be executed while focusing
        DENY_FUNC_LIVE_EXECUTION = {'pointScanner','parkScanner','centerScanner','measureScannerFrequencySweep'};  % Cell array of strings specifying functions for which a live execution is denied (during acqState = Focus)
    end    
    
    %% Lifecycle
    methods
        function obj = ResScan(hSI, simulated, name, legacymode)
            if nargin < 2 || isempty(simulated)
                simulated = false;
            end
            
            if nargin < 3 || isempty(name)
                name = 'Resonant';
            end
            
            if nargin > 3 && ~isempty(legacymode) && legacymode
                custMdfHeading = 'ResScan';
            else
                legacymode = false;
                custMdfHeading = ['ResScan (' name ')'];
            end
            
            obj = obj@scanimage.components.Scan2D(hSI,simulated,name,legacymode);
            obj = obj@most.HasMachineDataFile(true, custMdfHeading);
            
            %Initialize class data file (ensure props exist in file)
            obj.zprvEnsureClassDataFileProps();
            
            %Initialize the scan maps (from values in Class Data File)
            obj.loadClassData();
            
            %Verify mdf settings
            assert(isempty(obj.mdfData.beamDaqID) || (obj.mdfData.beamDaqID <= obj.hSI.hBeams.numInstances), 'ResScan: Invalid value for beamDaqID');
            
            %Construct sub-components
            % Open FPGA acquisition adapter
            obj.hAcq = scanimage.components.scan2d.resscan.Acquisition(obj,obj.simulated);
            
            % Open scanner control adapter
            obj.hCtl = scanimage.components.scan2d.resscan.Control(obj,obj.simulated);
            
            % Open trigger routing adapter
            obj.hTrig = scanimage.components.scan2d.resscan.Triggering(obj,obj.simulated);
            
            obj.numInstances = 1; % This has to happen _before_ any properties are set
            
            % initialize scanner frequency from mdfData
            obj.scannerFrequency = obj.mdfData.nominalResScanFreq;
            
            %Initialize sub-components
            obj.hAcq.frameAcquiredFcn = @(src,evnt)obj.frameAcquiredFcn;            
            obj.hAcq.initialize();
            obj.hCtl.initialize();
            
            obj.channelsFilter = 'Bessel';      % channels filter type; one of {'None','Elliptic','Bessel'}
                        
            %Initialize Scan2D props (not initialized by superclass)
            obj.channelsInputRanges = repmat(obj.channelsAvailableInputRanges(1),1,obj.channelsAvailable);
            obj.channelOffsets = zeros(1, obj.channelsAvailable);
            obj.channelsSubtractOffsets = true(1, obj.channelsAvailable);
        end
        
        function delete(obj)
            most.idioms.safeDeleteObj(obj.hTrig);
            most.idioms.safeDeleteObj(obj.hAcq);
            most.idioms.safeDeleteObj(obj.hCtl);
            
            obj.saveMaps();
        end
    end
    
    %% PROP ACCESS METHODS
    methods
        function set.channelOffsets(obj,val)
            if ~isempty(val)
                assert(numel(val) == obj.channelsAvailable, 'Number of elements must match number of physical channels.');
                lclSubtractOffset = cast(obj.channelsSubtractOffsets,obj.channelsDataType);
                for iter = 1:min(numel(val),numel(lclSubtractOffset))
                    fpgaVal(iter) = -val(iter) * lclSubtractOffset(iter);
                end
                obj.channelOffsets = val;
                obj.hAcq.hFpga.AcqParamLiveChannelOffsets = fpgaVal;
            end
        end
        
        function set.pixelBinFactor(obj,val)
            obj.errorPropertyUnSupported('pixelBinFactor',val);
        end
        
        function set.sampleRate(obj,val)
            obj.errorPropertyUnSupported('sampleRate',val,'set');
            
            %side effects
            obj.linePhase = obj.linePhase;
        end
        
        function val = get.sampleRate(obj)
            val = obj.hAcq.sampleRateInternal_;
        end
        
        function set.fpgaShutterOut(obj,val)
            validateattributes(val,{'logical' 'numeric'},{'binary' 'scalar'});
            obj.hAcq.hFpga.ShutterOutput = val;
        end
        
        function val = get.fpgaShutterOut(obj)
            val = obj.hAcq.hFpga.ShutterOutput;
        end
        
        function set.fpgaShutterOutTerm(obj,val)
            obj.hAcq.hFpga.ShutterTerminalOut = val;
        end
        
        function val = get.fpgaShutterOutTerm(obj)
            val = obj.hAcq.hFpga.ShutterTerminalOut;
        end
        
        function val = get.resonantScannerLastWrittenValue(obj)
           val = obj.hCtl.resonantScannerLastWrittenValue; 
        end
        
        function set.linePhaseStep(obj,val)
            obj.mdlDummySetProp(val,'linePhaseStep');
        end
        
        function val = get.linePhaseStep(obj)
            val = 1 / obj.sampleRate;
        end
        
        function val = get.multiChannel(obj)
            val = numel(obj.hSI.hChannels.channelsActive) > 1;
        end
        
        function val = get.digitalIODeviceType(obj)
            val = obj.hTrig.digitalIODeviceType;
        end
        
        function val = get.digitalIODaqName(obj)
            val = obj.hTrig.digitalIODaqName;
        end
        %         function val = get.trigNextInTermAllowed(obj)
        %             val = obj.allowedTriggerInputTerminals;
        %         end
        %
        %         function val = get.trigStopInTermAllowed(obj)
        %             val = obj.allowedTriggerInputTerminals;
        %         end
        
        function set.linePhaseMode(obj, v)
            assert(ismember(v, {'Next Lower' 'Next Higher' 'Nearest Neighbor' 'Interpolate'}), 'Invalid choice for linePhaseMode. Must be one of {''Next Lower'' ''Next Higher'' ''Nearest Neighbor'' ''Interpolate''}.');
            obj.linePhaseMode = v;
            if obj.mdlInitialized && obj.numInstances > 0
                obj.setClassDataVar('linePhaseMode',v,obj.classDataFileName);
            end
        end
        
        function set.enableContinuousFreqMeasurement(obj, v)
            if obj.componentUpdateProperty('enableContinuousFreqMeasurement',v)
                obj.enableContinuousFreqMeasurement = v;
                
                if v && strcmp(obj.hCtl.hTimerContinuousFreqMeasurement.Running,'off')
                    start(obj.hCtl.hTimerContinuousFreqMeasurement);
                else
                    stop(obj.hCtl.hTimerContinuousFreqMeasurement);
                end
            end
        end
        
        function set.keepResonantScannerOn(obj, v)
            obj.keepResonantScannerOn = v;
            if obj.mdlInitialized && obj.numInstances > 0
                obj.setClassDataVar('keepResonantScannerOn',v,obj.classDataFileName);
                if ~obj.active
                    obj.hCtl.resonantScannerActivate(v);
                end
            end
        end
        
        function set.resonantLimitedFovMode(obj,v)
            if ~v && ~obj.hCtl.xGalvoExists
                most.idioms.warn('Disabling resonant limited FOV mode is only supported for RGG scanners.');
                v = true;
            end
            
            obj.resonantLimitedFovMode_ = v;
            
            obj.setClassDataVar('resonantLimitedFovMode',v,obj.classDataFileName);
            
            if obj.hSI.hScan2D == obj && ~obj.hSI.hRoiManager.mroiEnable
                obj.hSI.hRoiManager.clearRoiGroupScannerCoordCache();
            end
        end
        
        function v = get.resonantLimitedFovMode(obj)
            v = obj.resonantLimitedFovMode_;
        end
        
        function set.useNonlinearResonantFov2VoltsCurve(obj,v)
            v = obj.validatePropArg('useNonlinearResonantFov2VoltsCurve',v);
            if obj.componentUpdateProperty('useNonlinearResonantFov2VoltsCurve',v)
                obj.useNonlinearResonantFov2VoltsCurve_ = v;
                obj.setClassDataVar('useNonlinearResonantFov2VoltsCurve',v,obj.classDataFileName);
                if v
                    if obj.mdlInitialized
                        obj.plotFov2VoltageCurve();
                    end
                    if abs((obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree) - obj.zzzResonantFov2Volts(1)) > 0.01
                        most.idioms.warn('FOV to voltage map value for FOV=1 is not consistent with mdf. Map may need to be reset.');
                    end
                end
                
                obj.updateLiveValues();
            end
        end
        
        function v = get.useNonlinearResonantFov2VoltsCurve(obj)
            v = obj.useNonlinearResonantFov2VoltsCurve_;
        end
        
        function sz = get.defaultRoiFovSize(obj)
            if obj.resonantLimitedFovMode
                rgs = obj.angularRange;
                sz = min(rgs) ./ rgs;
            else
                angle = min(obj.mdfData.resonantAngularRange, obj.mdfData.yGalvoAngularRange);
                sz = angle ./ obj.angularRange;
            end
        end
        
        function rg = get.angularRange(obj)
            if obj.resonantLimitedFovMode
                x = obj.mdfData.resonantAngularRange;
            else
                x = obj.mdfData.xGalvoAngularRange + obj.mdfData.resonantAngularRange;
            end
            rg = [x obj.mdfData.yGalvoAngularRange];
        end
    end
      
    %%% Abstract method implementations (scanimage.components.Scan2D)
    % AccessXXX prop API for Scan2D
    methods (Access = protected, Hidden)
        function val = accessScannersetPostGet(obj,~)
            if obj.hCtl.xGalvoExists
                val=obj.makeScannerSetDescriptionRGG();
            else
                val=obj.makeScannerSetDescriptionRG();
            end
        end
        
        function accessBidirectionalPostSet(obj,~)
%             %Side-effects                        
%             obj.linesPerFrame = obj.linesPerFrame;                  % make sure that linesPerFrame is even when bidirectional scanning
%             obj.flybackLinesPerFrame = obj.flybackLinesPerFrame;    % make sure that flybackLinesPerFrame is even when bidirectional scanning
%                                     
%             obj.hAcq.computeMask();
%             obj.hAcq.flagResizeAcquisition = true;
        end
        
        function val = accessLinePhasePreSet(obj,val)
            % line phase is measured in seconds
            samples = round((val) * obj.sampleRate);
            val = samples / obj.sampleRate; % round to closest possible value
            
            if ~isempty(obj.hCtl.resonantScannerLastWrittenValue) && ( ~obj.robotMode && ~obj.flagZoomChanged && obj.mdlInitialized && obj.hCtl.resonantScannerLastWrittenValue > 0 )
                %Only cache the linePhase vlaue when its values have been adjusted by the user
                obj.linePhaseMap(round(obj.hCtl.resonantScannerLastWrittenValue*1000)/1000) = val;
            end
            
            obj.flagZoomChanged = false;
        end
        
        function accessLinePhasePostSet(obj)
            obj.hAcq.fpgaUpdateLiveAcquisitionParameters('linePhaseSamples');
        end
        
        function val = accessLinePhasePostGet(obj,val)
            % No-op
        end
        
        function val = accessChannelsFilterPostGet(~,val)
            % no-op
        end
        
        function val = accessChannelsFilterPreSet(obj,val)
            switch lower(val)
                case {'bypass','none',''}
                    filterType = 0;
                case 'elliptic'
                    filterType = 1;
                case 'bessel'
                    filterType = 2;
                otherwise
                    assert(false,'Not a valid filter type: %s. Valid types are ''None'' ''Elliptic'' ''Bessel''',val);
            end
            
            userCommand = 1; % User command for filter settings (Refer to FlexRIO help)
            
            if ~obj.hAcq.simulated
                for channelNumber = 0:(obj.hAcq.adapterModuleChannelCount - 1)
                    status = obj.hAcq.sendAdapterModuleUserCommand(userCommand,channelNumber,filterType);
                    assert(status == 0,'Setting filter type for channel %d returned fpga error code %d',channelNumber,status);
                end
            end
        end
        
        function val = accessBeamClockDelayPreSet(~,val)            
            %no op
        end
        
        function accessBeamClockDelayPostSet(obj,~)            
            obj.hAcq.fpgaUpdateLiveAcquisitionParameters('beamClockDelay'); %The delay applies to the 'head', not 'tail' of the beam buffer
        end
        
        function val = accessBeamClockExtendPreSet(~,val)            
            %no op
        end
        
        function accessBeamClockExtendPostSet(obj,~)
            if obj.active
                obj.hSI.hBeams.updateBeamBufferAsync(true);
            end
        end
        
        function accessChannelsAcquirePostSet(obj,~)
            obj.hAcq.flagResizeAcquisition = true;
        end
        
        function val = accessChannelsInputRangesPreSet(obj,val)
            switch obj.hAcq.flexRioAdapterModuleName
                case {'NI5732','NI5734'}
                    for channelNumber = 1:obj.hAcq.adapterModuleChannelCount
                        channelRange = val{channelNumber};
                        validateattributes(channelRange,{'numeric'},{'numel', 2});
                        channelUpperLimit = channelRange(2);

                        % Execute user command
                        userCommand = 2; % User command for gain settings (Refer to FlexRIO help)
                        userData0 = channelNumber - 1; %channel Number on FPGA is zero-based
                        userData1 = obj.hAcq.CHANNEL_INPUT_RANGE_FPGA_COMMAND_DATA_MAP(channelUpperLimit);

                        obj.hAcq.sendNonBlockingAdapterModuleUserCommand(userCommand,userData0,userData1);

                        val{channelNumber} = channelRange;
                    end
                case 'NI5751'
                    % the input range of the 5751 is fixed at 2Vpp
                    channelRanges = {};
                    for channelNumber = 1:obj.hAcq.adapterModuleChannelCount
                        channelRanges{channelNumber} = [-1,1];
                    end
                    val = channelRanges;
                case 'NI517x'
                    obj.hAcq.configOscopeChannels(val);
                    
                otherwise
                    assert(false);
            end
        end
        
        function val = accessChannelsInputRangesPostGet(~,val)
            %No-op
        end
        
        function val = accessChannelsAvailablePostGet(obj,~)
            val = obj.hAcq.adapterModuleChannelCount;
        end
        
        function val = accessChannelsAvailableInputRangesPostGet(obj,~)
            upperLimits = obj.hAcq.ADAPTER_MODULE_AVAIL_INPUT_RANGES(obj.hAcq.flexRioAdapterModuleName);
            upperLimits = sort(upperLimits,'descend');
            numRanges = length(upperLimits);
            val = cell(1,numRanges);
            for i = 1:numRanges;
                val{i} = [-upperLimits(i) upperLimits(i)];
            end
        end
                     
        function val = accessFillFractionSpatialPreSet(obj,val)
            try
                scanimage.util.computeresscanmask(obj.scannerFrequency, obj.sampleRate, val, obj.hAcq.pixelsPerLine, false);
            catch
                most.idioms.warn('Attempted to set fill fraction too low.', val);
                val = obj.fillFractionSpatial;
            end
        end
                     
        function accessFillFractionSpatialPostSet(obj,~)
            obj.hAcq.computeMask();
        end
        
        function val = accessSettleTimeFractionPostSet(obj,val)
            obj.errorPropertyUnSupported('settleTimeFraction',val);
        end
        
        function val = accessFlytoTimePerScanfieldPostGet(obj,val)
            numScannerPeriods = ceil(val * obj.scannerFrequency);
            val = numScannerPeriods / obj.scannerFrequency;
        end
        
        function val = accessFlybackTimePerFramePostGet(obj,val)
            numScannerPeriods = ceil(val * obj.scannerFrequency);
            val = numScannerPeriods / obj.scannerFrequency;
        end
        
        function accessLogAverageFactorPostSet(obj,~)
            obj.hAcq.flagResizeAcquisition = true;
        end
        
        function accessLogFileCounterPostSet(obj,~)
            obj.hAcq.flagResizeAcquisition = true;
        end
        
        function accessLogFilePathPostSet(obj,~)
            obj.hAcq.flagResizeAcquisition = true;
        end
        
        function accessLogFileStemPostSet(obj,~)
            obj.hAcq.flagResizeAcquisition = true;
        end
        
        function accessLogFramesPerFilePostSet(obj,~)
            obj.hAcq.flagResizeAcquisition = true;
        end
        
        function accessLogFramesPerFileLockPostSet(obj,~)
            obj.hAcq.flagResizeAcquisition = true;
        end
        
        function accessLogHeaderStringPostSet(obj,~)
            obj.hAcq.flagResizeAcquisition = true;
        end
        
        function val = accessLogNumSlicesPreSet(obj,val)
            obj.hAcq.loggingNumSlices = val;
        end
        
        function val = accessTrigFrameClkOutInternalTermPostGet(obj,~)
            val = obj.hTrig.readInternalTerminalName('frameClockOut');
        end
        
        function val = accessTrigBeamClkOutInternalTermPostGet(obj,~)
            val = obj.hTrig.readInternalTerminalName('beamModifiedLineClockOut');
        end
        
        function val = accessTrigAcqOutInternalTermPostGet(obj,~)
            val = obj.hTrig.readInternalTerminalName('acqTriggerOut');
        end
        
        function val = accessTrigReferenceClkOutInternalTermPostGet(~,~)
            val = 'PXI_CLK10';
        end
        
        function val = accessTrigReferenceClkOutInternalRatePostGet(~,~)
            val = 10e6;
        end
        
        function val = accessTrigReferenceClkInInternalTermPostGet(~,~)
            val = 'PXI_CLK10';
        end
        function val = accessTrigReferenceClkInInternalRatePostGet(~,~)
            val = 10e6;
        end  
        function val = accessTrigAcqInTermAllowedPostGet(obj,~) 
            val = obj.hTrig.externalTrigTerminalOptions;
        end
        
        function val = accessTrigNextInTermAllowedPostGet(obj,~)
            val = obj.hTrig.externalTrigTerminalOptions;
        end
        
        function val = accessTrigStopInTermAllowedPostGet(obj,~)
            val = obj.hTrig.externalTrigTerminalOptions;
        end
             
        function val = accessTrigAcqEdgePreSet(obj,val)    
            obj.hTrig.acqTriggerOnFallingEdge = strcmp(val,'falling');
        end
        
        function accessTrigAcqEdgePostSet(~,~)
            % Nothing to do here
        end
        
        function val = accessTrigAcqInTermPreSet(obj,val)                        
            if isempty(val)
                obj.trigAcqTypeExternal = false;
            end
            obj.hTrig.acqTriggerIn = val;
        end
        
        function accessTrigAcqInTermPostSet(~,~)
            % Nothing to do here
        end
        
        function val = accessTrigAcqInTermPostGet(obj,~)
            val = obj.hTrig.acqTriggerIn;
        end
        
        function val = accessTrigAcqTypeExternalPreSet(obj,val)
            val = logical(val); % convert 'binaryflex' to 'logcial'
            
            if val
                obj.hAcq.hFpga.AcqTriggerType = 'External or Software';
                obj.hAcq.hFpga.StopTriggerType = 'External or Software';
                obj.hAcq.hFpga.AdvanceTriggerType = 'External or Software';
            else
                obj.hAcq.hFpga.AcqTriggerType = 'Software';
                obj.hAcq.hFpga.StopTriggerType = 'Software';
                obj.hAcq.hFpga.AdvanceTriggerType = 'Software';
            end                        
        end
        
        function accessTrigAcqTypeExternalPostSet(~,~)
             %No-op        
        end
        
        function val = accessTrigNextEdgePreSet(obj,val)
            obj.hTrig.nextFileMarkerOnFallingEdge = strcmp(val,'falling');
        end
        
        function val = accessTrigNextInTermPreSet(obj,val)
            obj.hTrig.nextFileMarkerIn = val;
        end
        
        function val = accessTrigNextStopEnablePreSet(~,~)
            val = true; % the FPGA can handle Next and Stop triggering at all times. no need to deactivate it               
        end
        
        function val = accessTrigStopEdgePreSet(obj,val)
            obj.hTrig.acqStopTriggerOnFallingEdge = strcmp(val,'falling');
        end
        
        function val = accessFunctionTrigStopInTermPreSet(obj,val)
            %termName = obj.allowedTriggerInputTerminalsMap(val); % qualify terminal name (e.g. DIO0.1 -> /FPGA/DIO0.1)
            obj.hTrig.acqStopTriggerIn = val;
        end
        
        function val = accessMaxSampleRatePostGet(obj,~)
            val = obj.sampleRate;
        end
        
        function accessScannerFrequencyPostSet(obj,~)
            obj.nomResPeriodTicks = floor(obj.sampleRate / obj.scannerFrequency);
            obj.resonantTimebaseTicksPerPeriod = floor(obj.resonantTimebaseNominalRate / obj.scannerFrequency);
            obj.scannerPeriodRTB = obj.resonantTimebaseTicksPerPeriod / obj.resonantTimebaseNominalRate;
        end
        
        function val = accessScannerFrequencyPostGet(~,val)
            % No op
        end
        
%         function val = accessRobotModePreSet(obj,val)
%             obj.hAcq.delayMaskComputation = val;
%         end

        function val = accessScanPixelTimeMeanPostGet(obj,~)
            if ~obj.active
                % if acq is active, let this occur automatically
                ppl = maxPixelsPerLine(obj.hSI.hRoiManager.currentRoiGroup);
                if isempty(ppl) || ppl < 4
                    val = nan;
                    return;
                end
                obj.hAcq.pixelsPerLine = ppl;
                obj.hAcq.computeMask();
            end
            val = (sum(obj.hAcq.mask(1:obj.hAcq.pixelsPerLine)) / obj.sampleRate) / obj.hAcq.pixelsPerLine;
            
            function pixels = maxPixelsPerLine(roiGroup)
            %% Returns the maximum number of pixels per line in the RoiGroup.
                pixels = max(arrayfun(@(roi)maxPixelsPerLine(roi),roiGroup.rois));
                
                function pixels = maxPixelsPerLine(roi)
                    %% Returns the maximum number of pixels per line in the RoiGroup.
                    if ~isempty(roi.scanfields)
                        pixels = max(arrayfun(@(scanfield) scanfield.pixelResolution(1),roi.scanfields));
                    else
                        pixels = 0;
                    end
                end
            end
        end
        
        function val = accessScanPixelTimeMaxMinRatioPostGet(obj,~)
            if isnan(obj.scanPixelTimeMean)
                val = nan;
            else
                maxPixelSamples = double(max(obj.hAcq.mask(1:obj.hAcq.pixelsPerLine)));
                minPixelSamples = double(min(obj.hAcq.mask(1:obj.hAcq.pixelsPerLine)));
                val = maxPixelSamples / minPixelSamples;
            end
        end
        
        function val = accessChannelsAdcResolutionPostGet(obj,~)
            val = obj.hAcq.ADAPTER_MODULE_ADC_BIT_DEPTH(obj.hAcq.flexRioAdapterModuleName);
        end
        
        function val = accessChannelsDataTypePostGet(~,~)
            val = 'int16';
        end
        
        % Component overload function
        function val = componentGetActiveOverride(obj,~)
            val = obj.hAcq.acqRunning();
        end
    end
    
    %% USER METHODS
    
    %%% Abstract methods realizations (scanimage.interfaces.Scan2D)
    methods
        
        % methods to issue software triggers
        % these methods should only be effective if specified trigger type
        % is 'software'
        function trigIssueSoftwareAcq(obj)
            if obj.componentExecuteFunction('trigIssueSoftwareAcq')
                obj.hAcq.generateSoftwareAcqTrigger();
            end
        end
        
        function trigIssueSoftwareNext(obj)
            if obj.componentExecuteFunction('trigIssueSoftwareNext')
                obj.hAcq.generateSoftwareNextFileMarkerTrigger();
            end
        end
        
        function trigIssueSoftwareStop(obj)
            if obj.componentExecuteFunction('trigIssueSoftwareStop')
                obj.hAcq.generateSoftwareAcqStopTrigger();
            end
        end
        
        function pointScanner(obj,fastDeg,slowDeg)
            % points the XY scanner to a position (units: degree)
            if obj.componentExecuteFunction('pointScanner',fastDeg,slowDeg)
                obj.hCtl.pointResAmplitudeDeg(fastDeg);
                obj.hCtl.pointGalvoDeg(slowDeg);
            end
        end
        
        function centerScanner(obj)
            % centers the XY scanner
            if obj.componentExecuteFunction('centerScanner')
                obj.hCtl.pointResAmplitudeDeg(0);
                obj.hCtl.centerGalvo();
            end
        end
        
        function parkScanner(obj)
            % parks the scanner, deactivates resonant scanner
            if obj.componentExecuteFunction('parkScanner')
                obj.hCtl.parkGalvo();
                if obj.mdlInitialized
                    obj.hCtl.resonantScannerActivate(obj.keepResonantScannerOn);
                end
            end
        end
        
        function updateLiveValues(obj)
            obj.hCtl.updateLiveValues();
            if obj.active && strcmpi(obj.hSI.acqState,'focus')
                obj.hAcq.bufferAcqParams(true);
            end
            obj.updateFov2VoltageCurvePlot();
        end
    end
    
    %%% Resonant scanning specific methods
    methods
        function resFreq = measureScannerFrequency(obj)
            if obj.componentExecuteFunction('measureScannerFrequency')
                
                if ~obj.active
                    obj.hCtl.resonantScannerActivate(true);
                elseif obj.enableContinuousFreqMeasurement
                    
                    if isempty(obj.lastLiveScannerFreqMeasTime)
                        fprintf('Continuous measurement is enabled but no reading has been made yet.\n');
                    else
                        v = obj.hCtl.resonantScannerLastWrittenValue;
                        resFreq = obj.liveScannerFreq;
                        
                        fprintf('Continuous measurement is enabled. Last sample was taken %.2f seconds ago for zoom voltage of %.3fV: %.2fHz\n',etime(clock,obj.lastLiveScannerFreqMeasTime),v,resFreq);
                        
                        obj.scanFreqMap(round(v*1000)/1000) = resFreq;
                    end
                    
                    return;
                end
                
                v = obj.hCtl.resonantScannerLastWrittenValue;
                fprintf('Measuring scanner frequency at zoom voltage of %.3fV...\n',v);
                
                % assumption: the scanner frequency should be settled after 2 seconds
                obj.hCtl.resonantScannerWaitSettle(max(2,obj.mdfData.resonantScannerSettleTime));
                
                resFreq = obj.hCtl.calibrateResonantScannerFreq();
                
                if ~(obj.active || obj.keepResonantScannerOn)
                    obj.hCtl.resonantScannerActivate(false);
                end
                
                if isnan(resFreq)
                    most.idioms.dispError('Failed to read scanner frequency. Period clock pulses not detected.\nVerify zoom control/period clock wiring and MDF settings.\n');
                else
                    fprintf('Scanner Frequency calibrated: %.2fHz\n',resFreq);
                    obj.scanFreqMap(round(v*1000)/1000) = resFreq;

                    if ~obj.active
                        %Side-effects
                        obj.scannerFrequency = resFreq;
                        obj.hAcq.computeMask();
                        obj.saveMaps(false,true);
                    end
                end
            end
        end
        
        function measureScannerFrequencySweep(obj,measPoints)
            if obj.componentExecuteFunction('measureScannerFrequencySweep')
                N = numel(measPoints);
                if N > 1
                    fprintf('Measuring scanner frequency at %d points. Press ctrl+c to cancel at any time.\n', N);
                end
                
                measPoints = round(measPoints*1000)/1000;
                
                onCleanup(@()cancelFunc);
                messy = false;
                
                for i = 1:N
                    messy = true;
                    fprintf('Measuring scanner frequency at amplitude of %.3fV (point %d of %d)... ', measPoints(i), i, N);
                    
                    obj.hCtl.resonantScannerActivate(true,measPoints(i));
                    obj.hCtl.resonantScannerWaitSettle(2);
                    resFreq = obj.hCtl.calibrateResonantScannerFreq();
                    obj.hCtl.resonantScannerActivate(false);
                    
                    if isnan(resFreq)
                        most.idioms.dispError('\nFailed to read scanner frequency. Period clock pulses not detected.\nVerify zoom control/period clock wiring and MDF settings.\n');
                        messy = false;
                        return;
                    else
                        fprintf('Done. Result: %.2fHz\n', resFreq);
                        messy = false;

                        obj.scanFreqMap(measPoints(i)) = resFreq;
                        obj.saveMaps(false, true);
                    end
                end
            end
            
            function cancelFunc
                if messy
                    obj.hCtl.resonantScannerActivate(false);
                    fprintf('Cancelled.\n');
                end
            end
        end
        
        function loadClassData(obj)
            ks = obj.getClassDataVar('linePhaseMap_ks',obj.classDataFileName);
            vs = obj.getClassDataVar('linePhaseMap_vs',obj.classDataFileName);
            if ~isa(ks,'double') || ~isa(vs,'double') || numel(ks) ~= numel(vs)
                most.idioms.warn('Line phase map from Class Data File contained unexpected data. Replacing with empty map.');
                obj.linePhaseMap = containers.Map('KeyType', 'double', 'ValueType', 'double');
            elseif isempty(ks)
                obj.linePhaseMap = containers.Map('KeyType', 'double', 'ValueType', 'double');
            else
                obj.linePhaseMap = containers.Map(ks,vs);
            end
            
            ks = obj.getClassDataVar('scanFreqMap_ks',obj.classDataFileName);
            vs = obj.getClassDataVar('scanFreqMap_vs',obj.classDataFileName);
            if ~isa(ks,'double') || ~isa(vs,'double') || numel(ks) ~= numel(vs)
                most.idioms.warn('Scan freq map from Class Data File contained unexpected data. Replacing with empty map.');
                obj.scanFreqMap = containers.Map('KeyType', 'double', 'ValueType', 'double');
            elseif isempty(ks)
                obj.scanFreqMap = containers.Map('KeyType', 'double', 'ValueType', 'double');
            else
                obj.scanFreqMap = containers.Map(ks,vs);
            end
            
            ks = obj.getClassDataVar('resFov2VoltsMap_ks',obj.classDataFileName);
            vs = obj.getClassDataVar('resFov2VoltsMap_vs',obj.classDataFileName);
            if ~isa(ks,'double') || ~isa(vs,'double') || numel(ks) ~= numel(vs)
                most.idioms.warn('Resonant voltage map from Class Data File contained unexpected data. Replacing with default map.');
                obj.resFov2VoltsMap = containers.Map([1 0],[obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree 0]);
            elseif isempty(ks)
                obj.resFov2VoltsMap = containers.Map([1 0],[obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree 0]);
            else
                nmax = obj.getClassDataVar('resFov2VoltsMap_nom_max',obj.classDataFileName);
                if obj.useNonlinearResonantFov2VoltsCurve && ~isempty(nmax) && abs(nmax - obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree) > 0.001
                    most.idioms.warn('The resonant scanner voltage settings have changed. The resonant calibration map must be reset for the change to take effect.');
                end
                obj.resFov2VoltsMap = containers.Map(ks,vs);
            end
            
            obj.resonantLimitedFovMode_ = obj.getClassDataVar('resonantLimitedFovMode',obj.classDataFileName);
            obj.useNonlinearResonantFov2VoltsCurve_ = obj.getClassDataVar('useNonlinearResonantFov2VoltsCurve',obj.classDataFileName);
            obj.keepResonantScannerOn = obj.getClassDataVar('keepResonantScannerOn',obj.classDataFileName);
            obj.linePhaseMode = obj.getClassDataVar('linePhaseMode',obj.classDataFileName);
        end
        
        
        function saveMaps(obj,savePhaseMap,saveFreqMap,saveVoltageMap)
            
            if nargin < 2
                savePhaseMap = true;
            end
            
            if nargin < 3
                saveFreqMap = true;
            end
            
            if nargin < 4
                saveVoltageMap = true;
            end
            
            if savePhaseMap
                obj.setClassDataVar('linePhaseMap_ks',cell2mat(obj.linePhaseMap.keys),obj.classDataFileName);
                obj.setClassDataVar('linePhaseMap_vs',cell2mat(obj.linePhaseMap.values),obj.classDataFileName);
            end
            
            if saveFreqMap
                obj.setClassDataVar('scanFreqMap_ks',cell2mat(obj.scanFreqMap.keys),obj.classDataFileName);
                obj.setClassDataVar('scanFreqMap_vs',cell2mat(obj.scanFreqMap.values),obj.classDataFileName);
            end
            
            if saveVoltageMap
                obj.setClassDataVar('resFov2VoltsMap_ks',cell2mat(obj.resFov2VoltsMap.keys),obj.classDataFileName);
                obj.setClassDataVar('resFov2VoltsMap_vs',cell2mat(obj.resFov2VoltsMap.values),obj.classDataFileName);
                nmax = obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree;
                obj.setClassDataVar('resFov2VoltsMap_nom_max',nmax,obj.classDataFileName);
            end
        end
        
        function plotFov2VoltageCurve(obj)
            if obj.componentExecuteFunction('plotFov2VoltageCurve')
                if(isempty(cell2mat(obj.resFov2VoltsMap.keys)))
                    obj.clearResFov2VoltsCal();
                end
                
                if ~obj.useNonlinearResonantFov2VoltsCurve
                    most.idioms.warn('''useNonlinearResonantFov2VoltsCurve'' is turned off. This curve is not actually being used for resonant mirror control. Set ''useNonlinearResonantFov2VoltsCurve'' to true to enable.')
                end
                
                fov = obj.hCtl.nextResonantFov;
                if fov >= 1.0001
                    fov = 1;
                end
                
                if most.idioms.isValidObj(obj.hCalFig)
                    figure(obj.hCalFig);
                    set(obj.hCalPlot, 'XData', cell2mat(obj.resFov2VoltsMap.keys), 'YData', cell2mat(obj.resFov2VoltsMap.values));
                    set(obj.hCalPlotPt, 'XData', fov, 'YData', obj.zzzResonantFov2Volts(fov));
                    set(obj.hCalFig, 'Visible', 'on');
                    set(findall(obj.hCalFig,'Tag','enableCurve'),'Value',obj.useNonlinearResonantFov2VoltsCurve);
                else
                    obj.hCalFig = figure('Name','Resonant FOV to Voltage Calibration Curve','NumberTitle','off','Color','White','MenuBar','none');
                    hmain=most.idioms.uiflowcontainer('Parent',obj.hCalFig,'FlowDirection','TopDown');
                        a = axes('Parent',hmain,'FontSize',12,'FontWeight','Bold');
                        obj.hCalPlot = plot(cell2mat(obj.resFov2VoltsMap.keys), cell2mat(obj.resFov2VoltsMap.values),'k.-','Parent',a,'MarkerSize',20,'LineWidth',2);
                        hold on;
                        obj.hCalPlotPt = plot(fov, obj.zzzResonantFov2Volts(fov),'ro','MarkerSize',10,'LineWidth',2,'Parent',a);
                        title('Resonant FOV to Voltage Calibration Curve','FontSize',12,'FontWeight','Bold');

                        xlabel('FOV','FontWeight','Bold');
                        xlim(a,[-.05 1.05]);

                        ylabel('Voltage (V)','FontWeight','Bold');
                        ylim(a,[-.1 5.1]);

                        grid(a,'on');
                        
                        bottompanel=uipanel('parent',hmain,'Title','Edit curve','FontSize',10);
                                set(bottompanel,'HeightLimits',[45 45]);
                                
                                hbottom = most.idioms.uiflowcontainer('Parent',bottompanel,'FlowDirection','LeftToRight');
                                uicontrol('parent',hbottom,'style','checkbox','string','Enable','Callback',@(src,evt)toggleEnable(),'Tag','enableCurve','Value',obj.useNonlinearResonantFov2VoltsCurve);
                                
                                uicontrol('parent',hbottom,'style','pushbutton','Callback',@(src,evt)adjustZoomFactor(1),'string','Zoom +');
                                uicontrol('parent',hbottom,'style','pushbutton','Callback',@(src,evt)adjustZoomFactor(-1),'string','Zoom -');
                                
                                uicontrol('parent',hbottom,'style','pushbutton','Callback',@(src,evt)obj.clearResFov2VoltsCal(),'string','Reset');
                                uicontrol('parent',hbottom,'style','pushbutton','Callback',@(src,evt)obj.createResFov2VoltsCalControlPoint(),'string','Add Point');
                                uicontrol('parent',hbottom,'style','pushbutton','Callback',@(src,evt)obj.removeResFov2VoltsCalControlPoint(),'string','Del Point');
                                
                                ad1 = uicontrol('parent',hbottom,'style','pushbutton','Callback',@(src,evt)obj.adjustResVoltageCal(-0.03),'string','<<');
                                ad2 = uicontrol('parent',hbottom,'style','pushbutton','Callback',@(src,evt)obj.adjustResVoltageCal(-0.005),'string','<');
                                ad3 = uicontrol('parent',hbottom,'style','pushbutton','Callback',@(src,evt)obj.adjustResVoltageCal(0.005),'string','>');
                                ad4 = uicontrol('parent',hbottom,'style','pushbutton','Callback',@(src,evt)obj.adjustResVoltageCal(0.03),'string','>>');
                                set([ad1 ad2 ad3 ad4],'WidthLimits',[40 40]);
                end
            end
            
            function adjustZoomFactor(val)
                obj.hSI.hRoiManager.scanZoomFactor = max(1,obj.hSI.hRoiManager.scanZoomFactor + val);
            end
            
            function toggleEnable()
                obj.useNonlinearResonantFov2VoltsCurve = ~obj.useNonlinearResonantFov2VoltsCurve;
            end
        end
        
        function createResFov2VoltsCalControlPoint(obj, fov, interpRange)
            if obj.componentExecuteFunction('createResFov2VoltsCalControlPoint')
                % if interp range is non zero, two additional control points are create at the specified distance from the main point
                % to limit the range of effect on the interpolated curve
                if nargin < 2 || isempty(fov)
                    fov = obj.hCtl.nextResonantFov;
                end
                
                if ~obj.useNonlinearResonantFov2VoltsCurve
                    most.idioms.warn('''useNonlinearResonantFov2VoltsCurve'' is turned off. This curve is not actually being used for resonant mirror control. Set ''useNonlinearResonantFov2VoltsCurve'' to true to enable.')
                end
                
                fov = round(fov*100000)/100000;
                obj.resFov2VoltsMap(fov) = obj.zzzResonantFov2Volts(fov);
                
                if nargin > 2 && ~isempty(interpRange) && interpRange > 0
                    fovh = round((fov + interpRange)*100000)/100000;
                    if fovh < obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree
                        obj.resFov2VoltsMap(fovh) = obj.zzzResonantFov2Volts(fovh);
                    end
                    fovl = round((fov - interpRange)*100000)/100000;
                    if fovl > 0
                        obj.resFov2VoltsMap(fovl) = obj.zzzResonantFov2Volts(fovl);
                    end
                end
                obj.updateFov2VoltageCurvePlot();
            end
        end
        
        function removeResFov2VoltsCalControlPoint(obj, fov)
            if obj.componentExecuteFunction('removeResFov2VoltsCalControlPoint')
                % if interp range is non zero, two additional control points are create at the specified distance from the main point
                % to limit the range of effect on the interpolated curve
                if nargin < 2 || isempty(fov)
                    fov = obj.hCtl.nextResonantFov;
                end
                
                if ~obj.useNonlinearResonantFov2VoltsCurve
                    most.idioms.warn('''useNonlinearResonantFov2VoltsCurve'' is turned off. This curve is not actually being used for resonant mirror control. Set ''useNonlinearResonantFov2VoltsCurve'' to true to enable.')
                end
                
                fov = round(fov*100000)/100000;
                if obj.resFov2VoltsMap.isKey(fov)
                    obj.resFov2VoltsMap.remove(fov);
                    obj.updateFov2VoltageCurvePlot();
                end
            end
        end
        
        function adjustResVoltageCal(obj, adj)
            if obj.componentExecuteFunction('adjustResVoltageCal')
                % adjust the voltage of the resonant scanner up or down for the
                % current desired FOV by [adj]%
                assert(logical(obj.useNonlinearResonantFov2VoltsCurve), '''useNonlinearResonantFov2VoltsCurve'' is turned off. Aspect ratio adjustment is not possible. Set ''useNonlinearResonantFov2VoltsCurve'' to true to enable.');
                
                fov = obj.hCtl.nextResonantFov;
                v = obj.zzzResonantFov2Volts(fov);
                v = v * (1+adj);
                
                if v > obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree
                    v = obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree;
                    most.idioms.warn('Adjustment out of range. Coercing to %.3fV.', v);
                end
                
                if v < 0
                    v = 0;
                    most.idioms.warn('Adjustment out of range. Coercing to %.3fV.', v);
                end
                
                fov = round(fov*100000)/100000;
                obj.resFov2VoltsMap(fov) = v;
                obj.updateLiveValues();
                obj.updateFov2VoltageCurvePlot();
            end
        end
        
        function fov2VoltageCal = exportFov2VoltageCal(obj,filename)
            if nargin < 2
                filename = '';
            end
            
            if nargout == 0 && isempty(filename)
                [name,path] = uigetfile('.mat','Choose file to save resonant voltage cal','fov2VoltageCal.mat');
                if name==0;return;end
                filename = fullfile(path,file);
            end
            
            if ~obj.useNonlinearResonantFov2VoltsCurve
                most.idioms.warn('''useNonlinearResonantFov2VoltsCurve'' is turned off. This curve is not actually being used for resonant mirror control. Set ''useNonlinearResonantFov2VoltsCurve'' to true to enable.')
            end
            
            fov2VoltageCal.angularRange = obj.mdfData.resonantAngularRange;
            fov2VoltageCal.voltsPerOpticalDegree = obj.mdfData.rScanVoltsPerOpticalDegree;
            fov2VoltageCal.fov = cell2mat(obj.resFov2VoltsMap.keys);
            fov2VoltageCal.angle = fov2VoltageCal.fov * obj.mdfData.resonantAngularRange;
            fov2VoltageCal.volts = cell2mat(obj.resFov2VoltsMap.values);
            
            if ~isempty(filename)
                save(filename,'fov2VoltageCal','-mat');
            end
        end
        
        function importFov2VoltageCal(obj,calOrFile)
            if nargin < 2 || isempty(calOrFile)
                [filename,pathname] = uigetfile('.mat','Choose file to load resonant voltage cal','fov2VoltageCal.mat');
                if filename==0;return;end
                calOrFile = fullfile(pathname,filename);
            end
            
            if ischar(calOrFile)
                fov2VoltageCal = load(filename,'-mat','fov2VoltageCal');
            elseif isstruct(calOrFile) && all(isfield(calOrFile, {'angularRange','voltsPerOpticalDegree','fov','volts'}))
                fov2VoltageCal = calOrFile;
            else
                error('Unkown input format.');
            end
            
            if ~obj.useNonlinearResonantFov2VoltsCurve
                most.idioms.warn('''useNonlinearResonantFov2VoltsCurve'' is turned off. This curve is not actually being used for resonant mirror control. Set ''useNonlinearResonantFov2VoltsCurve'' to true to enable.')
            end
            
            assignin('base','fov2VoltageCalBak',obj.exportFov2VoltageCal);
            
            % scale if zoom level 1 is at a different voltage
            fov2VoltageCal.fov = fov2VoltageCal.fov * (fov2VoltageCal.angularRange * fov2VoltageCal.voltsPerOpticalDegree)/(obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree);
            
            inds = fov2VoltageCal.fov(fov2VoltageCal.fov > 1);
            fov2VoltageCal.fov(inds) = [];
            fov2VoltageCal.volts(inds) = [];
            
            obj.resFov2VoltsMap = containers.Map(fov2VoltageCal.fov,fov2VoltageCal.volts);
            obj.updateFov2VoltageCurvePlot();
        end
        
        function clearZoomToLinePhaseCal(obj)
            %This function clears the scan phase map to default values.
            obj.linePhaseMap = containers.Map('KeyType', 'double', 'ValueType', 'double');
        end
        
        function clearZoomToScanFreqCal(obj)
            %This function clears the scan frequency map to default values.
            obj.scanFreqMap = containers.Map('KeyType', 'double', 'ValueType', 'double');
        end
        
        function clearResFov2VoltsCal(obj)
            if obj.componentExecuteFunction('clearResFov2VoltsCal')
                %This function clears the scan frequency map to default values.
                obj.resFov2VoltsMap = containers.Map([1 0],[obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree 0]);
                obj.updateFov2VoltageCurvePlot();
                
                if ~obj.useNonlinearResonantFov2VoltsCurve
                    most.idioms.warn('''useNonlinearResonantFov2VoltsCurve'' is turned off. This curve is not actually being used for resonant mirror control. Set ''useNonlinearResonantFov2VoltsCurve'' to true to enable.')
                end
            end
        end
    end
    
    %% FRIEND METHODS
    methods (Hidden)
        function periodClockInOverride(obj,val)
            obj.hTrig.overrideTriggers(end+1) = struct('trig','periodClockIn','val',val);
            obj.hTrig.periodClockIn = val;
        end
        
        function updateFov2VoltageCurvePlot(obj)
            if most.idioms.isValidObj(obj.hCalFig) && strcmp(get(obj.hCalFig, 'Visible'), 'on')
                obj.plotFov2VoltageCurve();
            end
        end
    end
    
    %% INTERNAL METHODS
    
    methods (Hidden)
        function reinitRoutes(obj)
            if obj.mdlInitialized
                obj.hTrig.hPrimaryIODaqDevice.reset();
                
                if obj.firstInit
                    obj.hTrig.initialize();
                    obj.firstInit = false;
                else
                    obj.hTrig.hRouteRegistry.reinitRoutes();
                end
            end
        end
        
        function frameAcquiredFcn(obj,src,evnt) %#ok<INUSD>
            if obj.active
                obj.stripeAcquiredCallback(obj,[]);
            end
        end
        
        function val = zzzEstimateLinePhase(obj,resonantOutputVolts)
            %Restrict resolution of map
            resonantOutputVolts = round(resonantOutputVolts*1000)/1000;
            
            if isempty(resonantOutputVolts) || isempty(keys(obj.linePhaseMap))
                %If there are no keys in the phase map or no voltage param set, default to zero.
                val = 0;
            else
                linePhaseMapArray = cell2mat(keys(obj.linePhaseMap));
                
                if ismember(resonantOutputVolts, linePhaseMapArray)
                    %If the resonant voltage is a key in the linePhaseMap, simply return its value.
                    val = obj.linePhaseMap(resonantOutputVolts);
                else
                    %If the resonant voltage is not a key in the linePhaseMap, then
                    %interpolate (or extrapolate) value from its nearest neighbors.
                    %Find the first key below this resonant voltage level.
                    lowKey = linePhaseMapArray(find(resonantOutputVolts>linePhaseMapArray,1,'last'));
                    %Find the first key above this resonant voltage level.
                    highKey = linePhaseMapArray(find(resonantOutputVolts<linePhaseMapArray,1,'first'));
                    if isempty(lowKey)
                        %If there is no key with a lower resonant voltage than
                        %the current one, return the val corresponding to the
                        %next lower known resonant voltage.
                        val = obj.linePhaseMap(highKey);
                    elseif isempty(highKey)
                        %If there is no key with a higher resonant voltage than
                        %the current one, return the val corresponding to the
                        %next higher known resonant voltage.
                        val = obj.linePhaseMap(lowKey);
                    else
                        %The usual case: There is a defined phase for resonant
                        %voltages greater than and less than he current one.
                        switch obj.linePhaseMode
                            % High and low are swapped because zoom is inverse to voltage
                            case 'Next Lower'
                                val = obj.linePhaseMap(highKey);
                            case 'Next Higher'
                                val = obj.linePhaseMap(lowKey);
                            case 'Nearest Neighbor'
                                if (highKey - resonantOutputVolts) > (resonantOutputVolts - lowKey)
                                    val = obj.linePhaseMap(lowKey);
                                else
                                    val = obj.linePhaseMap(highKey);
                                end
                            case 'Interpolate'
                                uniqueKeyVals = unique(linspace(lowKey,highKey));
                                
                                val = interp1(uniqueKeyVals, ...
                                    linspace(obj.linePhaseMap(lowKey), ...
                                    obj.linePhaseMap(highKey),numel(uniqueKeyVals)),resonantOutputVolts);
                        end
                    end
                end
            end
        end
        
        function val = zzzEstimateScanFreq(obj,resonantOutputVolts)
            %Restrict resolution of map
            resonantOutputVolts = round(resonantOutputVolts*1000)/1000;
            
            if isempty(resonantOutputVolts) || resonantOutputVolts == 0
                val = obj.mdfData.nominalResScanFreq;
            elseif isempty(keys(obj.scanFreqMap))
                %If there are no keys in the phase map, return default
                most.idioms.warn('Scanner frequency map is empty. For best results a measurement should be run.');
                val = obj.mdfData.nominalResScanFreq;
            else
                scanFreqMapArray = cell2mat(keys(obj.scanFreqMap));
                
                if ismember(resonantOutputVolts, scanFreqMapArray)
                    %If the resonant voltage is a key in the scanFreqMap, simply return its value.
                    val = obj.scanFreqMap(resonantOutputVolts);
                else
                    %If the resonant voltage is not a key in the scanFreqMap, then
                    %interpolate (or extrapolate) value from its nearest neighbors.
                    %Find the first key below this resonant voltage level.
                    lowKey = scanFreqMapArray(find(resonantOutputVolts>scanFreqMapArray,1,'last'));
                    %Find the first key above this resonant voltage level.
                    highKey = scanFreqMapArray(find(resonantOutputVolts<scanFreqMapArray,1,'first'));
                    if isempty(lowKey)
                        most.idioms.warn('Scanner frequency was not measured for a resonant amplitude at or below the current amplitude. Interpolation is not possible. Using nearest.');
                        %If there is no key with a lower resonant voltage than
                        %the current one, return the val corresponding to the
                        %next lower known resonant voltage.
                        val = obj.scanFreqMap(highKey);
                    elseif isempty(highKey)
                        most.idioms.warn('Scanner frequency was not measured for a resonant amplitude at or above the current amplitude. Interpolation is not possible. Using nearest.');
                        %If there is no key with a higher resonant voltage than
                        %the current one, return the val corresponding to the
                        %next higher known resonant voltage.
                        val = obj.scanFreqMap(lowKey);
                    else
                        %The usual case: There is a defined phase for
                        %zoomFactors greater than and less than the current
                        %one.
                        uniqueKeyVals = unique(linspace(lowKey,highKey));
                        
                        val = interp1(uniqueKeyVals, ...
                            linspace(obj.scanFreqMap(lowKey), ...
                            obj.scanFreqMap(highKey),numel(uniqueKeyVals)),resonantOutputVolts);
                    end
                end
            end
        end
        
        function val = zzzResonantFov2Volts(obj,fov)
            %Restrict resolution of map
            fov = round(fov*100000)/100000;
            assert((fov <= 1) && (fov >= 0), 'FOV out of range.');
            
            if obj.useNonlinearResonantFov2VoltsCurve
                if isempty(keys(obj.resFov2VoltsMap))
                    %If there are no keys in the phase map, return default
                    val = obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree * fov;
                else
                    resFov2VoltsMapArray = cell2mat(keys(obj.resFov2VoltsMap));
                    
                    if ismember(fov, resFov2VoltsMapArray)
                        %If the fov is a key in the resFov2VoltsMap, simply return its value.
                        val = obj.resFov2VoltsMap(fov);
                    else
                        %If the resonant fov is not a key in the resFov2VoltsMap, then
                        %interpolate (or extrapolate) value from its nearest neighbors.
                        
                        %Find the first key below this resonant voltage level.
                        lowKey = resFov2VoltsMapArray(find(fov>resFov2VoltsMapArray,1,'last'));
                        if isempty(lowKey)
                            lowKey = 0;
                            lowVal = 0;
                        else
                            lowVal = obj.resFov2VoltsMap(lowKey);
                        end
                        
                        %Find the first key above this resonant voltage level.
                        highKey = resFov2VoltsMapArray(find(fov<resFov2VoltsMapArray,1,'first'));
                        if isempty(highKey)
                            highKey = 1;
                            highVal = obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree;
                        else
                            highVal = obj.resFov2VoltsMap(highKey);
                        end
                        
                        val = interp1([lowKey highKey], [lowVal highVal], fov);
                    end
                end
            else
                val = obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree * fov;
            end
        end
    end
    
    methods (Hidden)%, Access = private)
        function configureFrameResolution(obj)
            zs=obj.hSI.hStackManager.zs; % generate planes to scan based on motor position etc
            
            [scanLines,flybackLines] = arrayfun(@(z)linesPerSlice(obj.currentRoiGroupScannerCoords,obj.scannerset,z),zs);
            scanLines = max(scanLines);
            linePixels = maxPixelsPerLine(obj.currentRoiGroupScannerCoords); % in resonant scanning, the FPGA cannot (yet) change pixels per lin
            flybackLines = max(flybackLines);
            
            obj.hAcq.pixelsPerLine = linePixels;
            obj.hAcq.linesPerFrame = scanLines;
            obj.hAcq.flybackLinesPerFrame = flybackLines;
            
            obj.hAcq.computeMask();
            obj.hAcq.flagResizeAcquisition = true;
            
            % local functions to operate on roiGroup
            function [scanLines,flybackLines] = linesPerSlice(roiGroup,scannerset,z)
                lineMask = acqActiveLineMask(roiGroup,scannerset,z);
                scanLines = numel(lineMask);
                [~,flybackTime] = roiGroup.transitTimes(scannerset,z);
                flybackLines = round(flybackTime * (1/scannerset.scanners{1}.scannerPeriod) * 2^scannerset.scanners{1}.bidirectionalScan);
            end
            
            function lineMask = acqActiveLineMask(roiGroup,scannerset,z)
                scanFields = roiGroup.scanFieldsAtZScannerCoordinates(scannerset,z);
                if(~isempty(scanFields))
                    % get transitLines
                    scanFieldsWithTransit = [{NaN} scanFields]; %pre- and ap- pend "park" to the scan field sequence to transit % the FPGA clock does not tick for the frame flyback, so we do not include the global flyback here
                    transitPairs = scanimage.mroi.util.chain(scanFieldsWithTransit); %transit pairs
                    transitTimes = cellfun(@(pair) scannerset.transitTime(pair{1},pair{2}),transitPairs);
                    linePeriods  = cellfun(@(sf)scannerset.linePeriod(sf),scanFields);
                    transitLines = round(transitTimes' ./ linePeriods);
                    
                    % get scanFieldLines
                    scanFieldLines = cellfun(@(sf)sf.pixelResolution(2),scanFields);
                    
                    lineMask = [];
                    for i = 1:length(scanFields)
                        lineMask(end+1:end+transitLines(i)) = false;
                        lineMask(end+1:end+scanFieldLines(i)) = true;
                    end
                else
                    lineMask = [];
                end
            end
            
            function pixels = maxPixelsPerLine(roiGroup)
            %% Returns the maximum number of pixels per line in the RoiGroup.
                pixels = max(arrayfun(@(roi)maxPixelsPerLine(roi),roiGroup.rois));
                
                function pixels = maxPixelsPerLine(roi)
                    %% Returns the maximum number of pixels per line in the RoiGroup.
                    if ~isempty(roi.scanfields)
                        pixels = max(arrayfun(@(scanfield) scanfield.pixelResolution(1),roi.scanfields));
                    else
                        pixels = 0;
                    end
                end
            end
        end
        
        function zprvEnsureClassDataFileProps(obj)
            obj.ensureClassDataFile(struct('linePhaseMap_ks',double([])),obj.classDataFileName);
            obj.ensureClassDataFile(struct('linePhaseMap_vs',double([])),obj.classDataFileName);
            
            obj.ensureClassDataFile(struct('scanFreqMap_ks',double([])),obj.classDataFileName);
            obj.ensureClassDataFile(struct('scanFreqMap_vs',double([])),obj.classDataFileName);
            
            obj.ensureClassDataFile(struct('resFov2VoltsMap_ks',double([])),obj.classDataFileName);
            obj.ensureClassDataFile(struct('resFov2VoltsMap_vs',double([])),obj.classDataFileName);
            nmax = obj.mdfData.resonantAngularRange * obj.mdfData.rScanVoltsPerOpticalDegree;
            obj.ensureClassDataFile(struct('resFov2VoltsMap_nom_max',nmax),obj.classDataFileName);
            
            obj.ensureClassDataFile(struct('resonantLimitedFovMode',true),obj.classDataFileName);
            obj.ensureClassDataFile(struct('useNonlinearResonantFov2VoltsCurve',false),obj.classDataFileName);
            obj.ensureClassDataFile(struct('keepResonantScannerOn',false),obj.classDataFileName);
            obj.ensureClassDataFile(struct('linePhaseMode','Nearest Neighbor'),obj.classDataFileName);
        end
        
        %% Resonant-Galvo Scanner
        function scannerset = makeScannerSetDescriptionRG(obj)            % 1. Describe the scanner set
            % Determine flyback time per frame
            if obj.hSI.hStackManager.isFastZ && strcmp(obj.hSI.hFastZ.waveformType, 'step')
                numScannerPeriods = ceil(obj.hSI.hFastZ.settlingTime * obj.scannerFrequency);
                flybackTime = max(obj.flybackTimePerFrame, numScannerPeriods / obj.scannerFrequency);
            else
                flybackTime = obj.flybackTimePerFrame;
            end
            
            % Determine scanner period to use for AO generation
            if obj.useResonantTimebase
                scannerPeriod = obj.scannerPeriodRTB;
            else
                scannerPeriod = 1/obj.scannerFrequency;
            end
            
            % Define Resonant Scanning Hardware.
            r = scanimage.mroi.scanners.Resonant(obj.mdfData.resonantAngularRange,...
                @obj.zzzResonantFov2Volts,...
                obj.bidirectional,...
                scannerPeriod,...
                obj.fillFractionSpatial,...
                obj.hCtl.rateAOSampClk);
            
            % Define Y-Galvo Scanning Hardware.
            assert(~isempty(obj.mdfData.yGalvoAngularRange),'yGalvoAngularRange is not defined in machine data file');
            gy = scanimage.mroi.scanners.Galvo(obj.mdfData.yGalvoAngularRange,...
                obj.mdfData.galvoVoltsPerOpticalDegreeY,...
                obj.flytoTimePerScanfield,... %flyto   time (seconds)
                flybackTime,...               %flyback time (seconds)
                obj.mdfData.galvoParkDegreesY,...
                obj.hCtl.rateAOSampClk);
            
            % Define beam hardware
            hBeams=obj.hSI.hBeams;
            if hBeams.numInstances && ~isempty(obj.mdfData.beamDaqID)
                beams = scanimage.mroi.scanners.Beams(hBeams.daqBeamIDs{obj.mdfData.beamDaqID},...
                    hBeams.maxSampleRate(obj.mdfData.beamDaqID),...
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
            
            % Create resonant galvo scannerset using hardware descriptions above
            scannerset=scanimage.mroi.scannerset.ResonantGalvo(obj.name,r,gy,beams,obj.fillFractionSpatial);
        end
        
        %% Resonant-Galvo-Galvo Scanner
        function scannerset = makeScannerSetDescriptionRGG(obj)            % 1. Describe the scanner set
            % Determine flyback time per frame
            if obj.hSI.hStackManager.isFastZ && strcmp(obj.hSI.hFastZ.waveformType, 'step')
                numScannerPeriods = ceil(obj.hSI.hFastZ.settlingTime * obj.scannerFrequency);
                flybackTime = max(obj.flybackTimePerFrame, numScannerPeriods / obj.scannerFrequency);
            else
                flybackTime = obj.flybackTimePerFrame;
            end
            
            % Determine scanner period to use for AO generation
            if obj.useResonantTimebase
                scannerPeriod = obj.scannerPeriodRTB;
            else
                scannerPeriod = 1/obj.scannerFrequency;
            end
            
            % Define Resonant Scanning Hardware.
            r = scanimage.mroi.scanners.Resonant(obj.mdfData.resonantAngularRange,...
                @obj.zzzResonantFov2Volts,...
                obj.bidirectional,...
                scannerPeriod,...
                obj.fillFractionSpatial,...
                obj.hCtl.rateAOSampClk);
            
            % Define X-Galvo Scanning Hardware.
            assert(~isempty(obj.mdfData.xGalvoAngularRange),'xGalvoAngularRange is not defined in machine data file');
            gx = scanimage.mroi.scanners.Galvo(obj.mdfData.xGalvoAngularRange,...
                obj.mdfData.galvoVoltsPerOpticalDegreeX,...
                obj.flytoTimePerScanfield,... %flyto   time (seconds)
                flybackTime,...               %flyback time (seconds)
                obj.mdfData.galvoParkDegreesX,...
                obj.hCtl.rateAOSampClk);
            
            % Define Y-Galvo Scanning Hardware.
            assert(~isempty(obj.mdfData.yGalvoAngularRange),'yGalvoAngularRange is not defined in machine data file');
            gy = scanimage.mroi.scanners.Galvo(obj.mdfData.yGalvoAngularRange,...
                obj.mdfData.galvoVoltsPerOpticalDegreeY,...
                obj.flytoTimePerScanfield,... %flyto   time (seconds)
                flybackTime,...               %flyback time (seconds)
                obj.mdfData.galvoParkDegreesY,...
                obj.hCtl.rateAOSampClk);
            
            % Define beam hardware
            hBeams=obj.hSI.hBeams;
            if hBeams.numInstances && ~isempty(obj.mdfData.beamDaqID)
                beams = scanimage.mroi.scanners.Beams(hBeams.daqBeamIDs{obj.mdfData.beamDaqID},...
                    hBeams.maxSampleRate(obj.mdfData.beamDaqID),...
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
            
            % Create resonant galvo galvo scannerset using hardware descriptions above
            scannerset=scanimage.mroi.scannerset.ResonantGalvoGalvo(obj.name,r,gx,gy,beams,obj.fillFractionSpatial);
            scannerset.resonantLimitedFovMode = obj.resonantLimitedFovMode;
        end
    end
    
    %%% Abstract methods realizations (scanimage.interfaces.Scan2D)    
    methods (Hidden)
        function arm(obj)
            resAmplitude = obj.hCtl.nextResonantVoltage;
            
            if resAmplitude > 0.0001
                obj.hCtl.resonantScannerActivate(true, resAmplitude);
                
%                 if obj.hCtl.getRemainingResSettlingTime() < 0.00001
%                     newFreq = obj.hCtl.calibrateResonantScannerFreq();
%                 else
                    newFreq = obj.zzzEstimateScanFreq(resAmplitude);
%                 end
                
                % avoid pointless change
                if (abs(newFreq - obj.scannerFrequency) / obj.scannerFrequency) > 0.00001
                    obj.scannerFrequency = newFreq;
                end
            end
        end
        
        function data = acquireSamples(obj,numSamples)
            if obj.componentExecuteFunction('acquireSamples',numSamples)
                data = zeros(numSamples,obj.channelsAvailable,obj.channelsDataType); % preallocate data
                if ~obj.mdfData.photonCountingEnable
                    for i = 1:numSamples
                        data(i,:) = obj.hAcq.rawAdcOutput(1,1:obj.channelsAvailable);
                    end
                end
            end
        end
        
        function signalReadyReceiveData(obj)
            obj.hAcq.signalReadyReceiveData();
        end
                
        function [success,stripeData] = readStripeData(obj)
            % remove the componentExecute protection for performance
            %if obj.componentExecuteFunction('readStripeData')
                [success,stripeData] = obj.hAcq.readStripeData();
                if stripeData.endOfAcquisitionMode
                    obj.abort(); %self abort if acquisition is done
                end
            %end
        end
        
        function calibrateLinePhase(obj,chanIdx)
            
            %just a calculation for how many frames we are gonna have to do
            avgFactor = obj.hSI.hDisplay.displayRollingAverageFactor;
            fineAdjustRangeFactor = 1.1;
            fineRange = obj.scanPixelTimeMean*fineAdjustRangeFactor*2;
            bigCoarseAdjustments = 4;
            coarseAdjustments = 3;
            N = numel(0:obj.linePhaseStep:fineRange)+coarseAdjustments+bigCoarseAdjustments;
            
            maxSteps = 800;
            
            cont = true;
            oldphase = obj.linePhase;
            hWb = waitbar(0,'Coarse Adjustment...','CreateCancelBtn',@(varargin)cancel());
            
            try
                bigCoarseAdjust();
                
                cDone = 0;
                while cDone < coarseAdjustments
                    coarseAdjust(true);
                    waitForFrames(avgFactor);
                    cDone = cDone + 1;
                    updateCheckWaitbar(true,(cDone+bigCoarseAdjustments)/N,'Coarse Adjustment...');
                end
                
                fineAdjust();
                delete(hWb);
            catch ME
                delete(hWb);
                ME.rethrow;
            end
            
            function cancel
                cont = false;
            end
            
            function updateCheckWaitbar(updt,pct,msg)
                if ~cont
                    obj.linePhase = oldphase;
                    delete(hWb);
                    return;
                elseif updt
                    waitbar(pct, hWb, msg);
                end
            end
            
            function im = getImage()
                %get image from every roi
                roiDatas = obj.hSI.hDisplay.lastFrame.roiData;
                for i = numel(roiDatas):-1:1
                    imData{i,1} = roiDatas{i}.imageData{chanIdx}{1};
                    if roiDatas{i}.transposed
                        imData{i,1} = imData{i,1}';
                    end
                end
                im = double(cell2mat(imData));
            end
            
            function [im1, im2] = deinterlaceImage(im)
                im1 = im(1:2:end,:);
                im2 = im(2:2:end,:);
            end
            
            function [bestoff, bestcorr] = xcor(im1,im2,singlePoint)
                if nargin < 3 || isempty(singlePoint)
                    singlePoint = false;
                end
                
                corrs = abs(most.mimics.xcorrcirc(im1,im2,[],2));
                corrs = mean(corrs,1);
                
                midpt = floor(size(im1,2)/2) + 1;
                if singlePoint
                    bestcorr = corrs(midpt);
                    bestoff = 0;
                else
                    [bestcorr, ib] = max(corrs);
                    bestoff = ib - midpt;
                end
            end
            
            function bigCoarseAdjust()
                phases = obj.linePhaseStep*(-maxSteps:(1600/(bigCoarseAdjustments-1)):maxSteps);
                
                for i = numel(phases):-1:1
                    obj.linePhase = phases(i);
                    waitForFrames(avgFactor);
                    bestcorr(i) = coarseAdjust(false);
                    updateCheckWaitbar(true,(1+numel(phases)-i)/N,'Coarse Adjustment...');
                end
                
                [~,i] = max(bestcorr);
                obj.linePhase = phases(i);
                oldphase = obj.linePhase;
            end
            
            function bestcorr = coarseAdjust(saveOld)
                im = getImage();
                [im1, im2] = deinterlaceImage(im);
                [pixOff, bestcorr] = xcor(im1,im2);
                if pixOff ~= 0
                    newPhs = obj.linePhase + pixOff * obj.scanPixelTimeMean * 0.45;
                    
                    if (newPhs > obj.linePhaseStep * maxSteps) || (newPhs < obj.linePhaseStep * -maxSteps)
                        %solution is too far off. disqualify it from big
                        %coarse competition
                        bestcorr = -inf;
                    else
                        obj.linePhase = newPhs;
                    end
                    
                    if saveOld
                        oldphase = obj.linePhase;
                    end
                end
            end
            
            
            
            function waitForFrames(N)
                t = tic();
                while toc(t) < (obj.hSI.hRoiManager.scanFramePeriod*N*1.1)
                    pause(.001);
                    updateCheckWaitbar(false);
                end
                
                fr = obj.hSI.hDisplay.lastFrameNumber;
                
                t = tic();
                while fr == obj.hSI.hDisplay.lastFrameNumber
                    pause(.01);
                    if toc(t) > obj.hSI.hRoiManager.scanFramePeriod * 5
                        obj.linePhase = oldphase;
                        error('Timed out waiting for a new frame.');
                    else
                        updateCheckWaitbar(false);
                    end
                end
            end
            
            function fineAdjust()
                minphase = oldphase - obj.scanPixelTimeMean*fineAdjustRangeFactor;
                maxphase = oldphase + obj.scanPixelTimeMean*fineAdjustRangeFactor;
                phases = minphase:obj.linePhaseStep:maxphase;
                
                dec = 0;
                
                corrs = -inf(1,numel(phases));
                
                for i = 1:numel(phases)
                    obj.linePhase = phases(i);
                    waitForFrames(avgFactor);
                    
                    updateCheckWaitbar(true,(bigCoarseAdjustments+coarseAdjustments+i)/N,'Fine Adjustment...');
                    
                    im = getImage();
                    [im1,im2] = deinterlaceImage(im);
                    [~, corrs(i)] = xcor(im1,im2,true);
                    
                    if (i > 1) && (corrs(i) < corrs(i-1))
                        dec = dec+1;
                        if dec > 3
                            %correlation got worse 4 times in a row. This is a local maxima. We are done here.
                            break;
                        end
                    else
                        dec = 0;
                    end
                end
                
                [~,i] = max(corrs);
                obj.linePhase = phases(i);
            end
        end
    end
    
    %%% Abstract methods realizations (scanimage.interfaces.Component)
    methods (Access = protected, Hidden)

        function componentStart(obj)
            assert(~obj.robotMode);
            obj.independentComponent = false;
            
            obj.configureFrameResolution();
            
            obj.hCtl.start();
            obj.hAcq.start();
            
            obj.flagZoomChanged = false;
            obj.linePhase_ = [];
        end
        
        function componentAbort(obj,soft)
            if nargin < 2 || isempty(soft)
                soft = false;
            end
            
            obj.hAcq.abort();
            obj.hCtl.stop(soft);
            
            obj.saveMaps();
            
            obj.flagZoomChanged = false;
            obj.linePhase_ = [];
            obj.independentComponent = true;
        end
        
        
        function fillFracTemp = fillFracSpatToTemp(~,fillFracSpat)
            fillFracTemp = 2/pi * asin(fillFracSpat);
        end
        
        function fillFracSpat = fillFracTempToSpat(~,fillFracTemp)
            fillFracSpat = cos( (1-fillFracTemp) * pi/2 );
        end
    end          
    
    %% FRIEND EVENTS
    events (Hidden, ListenAccess = {?scanimage.interfaces.Class},NotifyAccess = public); % for some reason NotifyAccess = {?scanimage.components.scan2d.resscan.Control} does not work
        resonantScannerOutputVoltsUpdated;
    end
    
end

function s = zlclAppendDependsOnPropAttributes(s)
s.useNonlinearResonantFov2VoltsCurve = struct('Classes','binaryflex');
end


%--------------------------------------------------------------------------%
% ResScan.m                                                                %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
