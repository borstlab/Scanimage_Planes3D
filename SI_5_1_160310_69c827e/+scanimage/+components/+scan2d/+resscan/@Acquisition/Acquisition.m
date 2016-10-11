classdef Acquisition < scanimage.interfaces.Class
    
    %% FRIEND  PROPS    
    
    %%% Key knobs for Acquisition functionality
    properties (Hidden, Access={?scanimage.interfaces.Class})        
        frameAcquiredFcn;           % Callback function to be executed when a frame is acquired
    end
    
    %%% Knobs for optional modulation of Acquisition functionality
    properties (Hidden)
        frameTagging = true;        % Activates frame tagging (now always active - 4/21/14); MEX code relies on frameTagging = true (4/16/2015)
        reverseLineRead = false;    % flip the image horizontally
        channelsInvert = false;     % specifies if the digitizer inverts the channel values
        
        simulatedFramePeriod = 33;  % Frame Period (in ms) at which to issue frames in simulated mode.
        
        pixelsPerLine = 512;
        linesPerFrame = 512;
        flybackLinesPerFrame = 16;
    end
    
    %%% Knobs for testing/debug
    properties (Hidden)
        simulated=false;
        debugOutput = false;        
        dummyData = false;
    end        
    
    %%% Flag props for use by external classes to schedule or defer Resize and Mask computations, respectively
    properties  (Hidden, Access={?scanimage.interfaces.Class}, AbortSet)      
        flagResizeAcquisition = true; % After startup the frame copier needs to be initialized 
        delayMaskComputation = false; % Flag to use during robot mode operations
    end
    
    %%% Read-only props
    properties (Hidden, GetAccess={?scanimage.interfaces.Class}, SetAccess=private, Dependent)
        adapterModuleChannelCount;
        rawAdcOutput;                % returns up-to-date Adc data without processing. can be queried at any time
        
        periodsPerFrame;
    end
        
    %% INTERNAL PROPS
    
    %Immutable props
    properties (Hidden,SetAccess = immutable)
        hScan;
        hFpga;
        
        fpgaFifoNumberSingleChan;
        fpgaFifoNumberMultiChan;
        fpgaFifoNumberAuxData;
        fpgaSystemTimerLoopCountRegister;
        
        flexRioAdapterModuleName;
    end

    properties (Hidden, SetAccess = private)  
        acqRunning = false;
        
        framesAcquired = 0;                     % total number of frames acquired in the acquisition mode
        acqCounter = 0;                         % number of acqs acquired
        lastEndOfAcquisition = 0;               % total frame number when the last end of acquisition flag was received
        epochAcqMode = [];                      % string, time of the acquisition of the acquisiton of the first pixel in the current acqMode; format: output of datestr(now) '25-Jul-2014 12:55:21'
        
        tagSizeFifoElements;                    % Number of FIFO elements for the tag (0 for frameTagging == 0)
                
        flagUpdateMask = true;                  % After startup the mask needs to be sent to the FPGA
        
		acqParamBuffer = struct();              % buffer holding frequently used parameters to limit parameter recomputation
        
        externalSampleClock = false;            % indicates if external/internal sample rate is used
        
        mexInit = false;
        AMCmdSent = false;
        
        sampleRateInternal_ = 0;
    end
    
    properties (Hidden, Dependent)
        scanFrameRate;               % number of frames per second
        dataRate;                    % the theoretical dataRate produced by the acquisition in MB/s
        dcOvervoltage;               % true if input voltage range is exceeded. indicates that coupling changed to AC to protect ADC        
        
        estimatedPeriodClockDelay;   % delays the start of the acquisition relative to the period trigger to compensate for line fillfractionSpatial < 1
        
        loggingEnable;               % accessed by MEX function
        channelsActive;              % accessed by MEX function
        channelsDisplay;             % accessed by MEX function
        channelsSave;                % accessed by MEX function
    end
    
    %%% Dependent properties computing values needed to pass onto FPGA API
    properties (Hidden, Dependent)
        linePhaseSamples;
        triggerHoldOff;
        beamTiming;        
        
        auxTriggersEnable;          % accessed by MEX
        
        I2CEnable;                  % accessed by MEX
        I2C_STORE_AS_CHAR;          % accessed by MEX
    end    
    
    %%% Properties made available to MEX interface, e.g. for logging and/or frame copy threads
    properties (Hidden, SetAccess = private)
        frameSizePixels;        %Number of Pixels in one frame (not including frame tag)
        frameSizeBytes;         %Number of Bytes in one frame (frame + optional frame tag)
        frameSizeFifoElements;  %Number of FIFO elements for one frame (frame + optional frame tag)
        
        fpgaFifoAuxDataActualDepth; %Number of elements the aux trigger can hold. Can differ from FIFO_ELEMENT_SIZE_AUX_TRIGGERS
    end
    
    %%% Mask params
    properties (Hidden, SetAccess=private)
        mask; %Array specifies samples per pixel for each resonant scanner period
        maskParams; %struct of params pertaining to mask        
    end
    
    properties (Hidden,Dependent)
        fifoSizeFrames;                     %number of frames the DMA FIFO can hold; derived from fifoSizeSeconds
        
        %name constant; accessed by MEX interface
        frameQueueLoggerCapacity;           %number of frames the logging frame queue can hold; derived from frameQueueSizeSeconds
        frameQueueMatlabCapacity;           %number of frames the matlab frame queue can hold; derived from frameQueueLoggerCapacity and framesPerStack
    end

    
    %% CONSTANTS
    properties (Hidden, Constant)
        ADAPTER_MODULE_MAP = containers.Map({278099318,278099319,278099349,278099186},{'NI5732','NI5733','NI5734','NI5751'});
        ADAPTER_MODULE_CHANNEL_COUNT = containers.Map({'NI5732','NI5733','NI5734','NI5751','NI517x'},{2,2,4,4,4});
        ADAPTER_MODULE_SAMPLING_RATE_MAP = containers.Map({'NI5732','NI5733','NI5734','NI5751','NI517x'},{80e6,120e6,120e6,50e6,125e6});
        ADAPTER_MODULE_SAMPLING_RATE_RANGE_MAP = containers.Map({'NI5732','NI5733','NI5734','NI5751','NI517x'},{[20e6 80e6],[50e6 120e6],[50e6 120e6],[50e6 50e6],[20e6 125e6]});        
        ADAPTER_MODULE_TRIGGER_TO_ADC_DELAY = containers.Map({'NI5732','NI5733','NI5734','NI5751','NI517x'},{16,16,16,0,0}); % TODO: Evaluate trigger delay for NI5751 and NI517x
        ADAPTER_MODULE_ADC_BIT_DEPTH = containers.Map({'NI5732','NI5733','NI5734','NI5751','NI517x'},{14,16,16,14,14});
        ADAPTER_MODULE_AVAIL_INPUT_RANGES = containers.Map({'NI5732','NI5733','NI5734','NI5751','NI517x'},{[1,0.5,0.25],[1,0.5,0.25],[1,0.5,0.25],[1,0.5,0.25],[2.5,1,0.5,0.1]});
        CHANNEL_INPUT_RANGE_FPGA_COMMAND_DATA_MAP = containers.Map({1,0.5,0.25},{0,1,2});
        
        FRAME_TAG_SIZE_BYTES = 32;
        
        FPGA_SYS_CLOCK_RATE = 200e6;        %Hard coded on FPGA[Hz]
        
        TRIGGER_HEAD_PROPERTIES = {'triggerClockTimeFirst' 'triggerTime' 'triggerFrameStartTime' 'triggerFrameNumber'};
        CHANNELS_INPUT_RANGES = {[-1 1] [-.5 .5] [-.25 .25]};
        
        HW_DETECT_POLLING_INTERVAL = 0.1;   %Hardware detection polling interval time (in seconds)
        HW_DETECT_TIMEOUT = 5;              %Hardware detection timeout (in seconds)
        HW_POLLING_INTERVAL = 0.01;         %Hardware polling interval time (in seconds)
        HW_TIMEOUT = 0.5;                   %Hardware timeout (in seconds)

        FIFO_SIZE_SECONDS = 1;              %time worth of frame data the DMA Fifo can hold
        FIFO_SIZE_LIMIT_MB = 250;           %limit of DMA FIFO size in MB
        FIFO_ELEMENT_SIZE_BYTES_MULTI_CHAN = 8;
        FIFO_ELEMENT_SIZE_BYTES_SINGLE_CHAN = 2;
        
        FRAME_QUEUE_LOGGER_SIZE_SECONDS = 1;     %time worth of frame data the logger frame queue can hold
        FRAME_QUEUE_LOGGER_SIZE_LIMIT_MB = 250;  %limit of logger frame queue size in MB
        
        FRAME_QUEUE_MATLAB_SIZE_SECONDS = 0.3;   %time worth of frame data the matlab frame queue can hold
        FRAME_QUEUE_MATLAB_SIZE_LIMIT_MB = 250;  %limit of matlab frame queue size in MB
        
        FIFO_ELEMENT_SIZE_AUX_DATA = 1000000;
        
        LOG_TIFF_HEADER_EXPANSION = 1000;   % (number of characters) the length of the Tiff Header Image Description is the length of logHeaderString + LOG_TIFF_HEADER_EXPANSION; values that are dynamically attached to the TIFF Header Description must fit into a string of length LOG_TIFF_HEADER_EXPANSION
        MEX_SEND_EVENTS = true;
        
        SYNC_DISPLAY_TO_VOLUME_RATE = true; % ensure that all consecutive slices within one volume are transferred from Framecopier. drop volumes instead of frames
    end
    
    %% Lifecycle
    methods
        function obj = Acquisition(hScan,simulated)
            if nargin < 1 || isempty(hScan) || ~isvalid(hScan)
                hScan = [];
            end
            
            if nargin < 2 || isempty(simulated)
                obj.simulated = hScan.simulated;
            else
                obj.simulated = simulated;
            end
            
            obj.hScan = hScan;
            
            obj.dispDbgMsg('Initializing Object & Opening FPGA session');
            
            % Determine bitfile name
            pathToBitfile = [fileparts(which('scanimage')) '\+scanimage\FPGA\FPGA Bitfiles\Microscopy'];

            if ~isempty(obj.hScan.mdfData.fpgaModuleType)
                pathToBitfile = [pathToBitfile ' ' obj.hScan.mdfData.fpgaModuleType];
            end

            if ~isempty(obj.hScan.mdfData.digitizerModuleType)
                pathToBitfile = [pathToBitfile ' ' obj.hScan.mdfData.digitizerModuleType];
            end

            pathToBitfile = [pathToBitfile '.lvbitx'];
            assert(logical(exist(pathToBitfile, 'file')), 'The FPGA and digitizer combination specified in the machine data file is not currently supported.');
            
            if strncmp(obj.hScan.mdfData.fpgaModuleType, 'NI517', 5)
                dabs.ni.oscope.clearSession
                err = dabs.ni.oscope.startSession(obj.hScan.mdfData.rioDeviceID,pathToBitfile);
                assert(err == 0, 'Error when attempting to connect to NI 517x device. Code = %d', err);
            end
            
            obj.hFpga = dabs.ni.rio.NiFPGA(pathToBitfile,obj.simulated);

            if (~obj.simulated)
                try
                    obj.hFpga.openSession(obj.hScan.mdfData.rioDeviceID);
                catch ME
                    error('Scanimage:Acquisition',['Failed to start FPGA. Ensure the FPGA and digitizer module settings in the machine data file match the hardware.\n' ME.message]);
                end
            end
            
            assert(isprop(obj.hFpga,'fifo_SingleChannelToHostI16')...
                && isprop(obj.hFpga,'fifo_MultiChannelToHostU64') ...
                && isprop(obj.hFpga,'fifo_AuxDataToHostU64'), ...
                'Expected FIFO objects not found for loaded FPGA module bitfile');
            
            %Hard-Reset FPGA. This brings the FPGA in a known state after an aborted acquisition
            obj.fpgaReset();
            
            obj.flexRioAdapterModuleName = obj.fpgaDetectAdapterModule();
            obj.configureAdapterModuleSampleClock();
            obj.resetDcOvervoltage(); % configure for DC coupling
            
            %Store FPGA device FIFO names. The names of the FIFO are parsed
            %from the bitfile, so they can change when the FPGA code is
            %modified. Storing the parameters here enables us to change the
            %names in Matlab without having to recompile the MEXfunction
            obj.fpgaFifoNumberSingleChan  = obj.hFpga.fifo_SingleChannelToHostI16.fifoNumber;
            obj.fpgaFifoNumberMultiChan   = obj.hFpga.fifo_MultiChannelToHostU64.fifoNumber;
            obj.fpgaFifoNumberAuxData = obj.hFpga.fifo_AuxDataToHostU64.fifoNumber;
            registerSysTimeProps = obj.hFpga.registerMap('SystemTimerLoopCount');
            obj.fpgaSystemTimerLoopCountRegister = registerSysTimeProps.offset;
            
            %Initialize MEX-layer interface
            ResonantAcqMex(obj,'init');
            obj.mexInit = true;
        end
        
        function delete(obj)
            if obj.acqRunning
                obj.abort();
            end
            
            if obj.mexInit
                ResonantAcqMex(obj,'delete');   % This will now unlock the mex file ot allow us to clear it from Matlab
            end
            clear('ResonantAcqMex'); % unload mex file
            
            most.idioms.safeDeleteObj(obj.hFpga);
        end
        
        function initialize(obj)
            %Initialize Mask
            obj.computeMask();
            
            %Set basic channel properties
            obj.channelsInvert = obj.hScan.mdfData.channelsInvert;
            
            %Initialize some fpga defaults
            period = obj.sampleRateInternal_ / obj.hScan.mdfData.nominalResScanFreq;
            obj.hFpga.MaxResonantPeriodTicks = floor(period*1.1);
            obj.hFpga.MinResonantPeriodTicks = floor(period*0.9);
            obj.hFpga.SettlingPeriods = 100;
            
            %photon counting
            obj.hFpga.PhotonCountingEnable = obj.hScan.mdfData.photonCountingEnable;
        end
    end          
    
    
    %% PROP ACCESS METHODS
    methods
        function resetDcOvervoltage(obj)
            % sets adapter module to DC coupling mode
            switch obj.flexRioAdapterModuleName
                case 'NI5751'
                    % 5751 does not support setting channels coupling
                    % (always runs in DC mode)
                case 'NI517x'
                    obj.configOscopeChannels();
                otherwise
                    setCoupling573x('AC'); % setting coupling mode to AC clears overvoltageStatus
                    setCoupling573x('DC');
            end
            
            %Helper function
            function setCoupling573x(mode)
                switch upper(mode)
                    % 0 = AC coupling, nonzero = DC coupling
                    case 'AC'
                        userData1 = 0;
                    case 'DC'
                        userData1 = 1;
                    otherwise
                        assert(false);
                end
                
                for channelNumber = 0:(obj.adapterModuleChannelCount-1)
                    % Execute user command
                    userCommand = 3; % User command for coupling settings (Refer to FlexRIO help)
                    userData0 = channelNumber; %channel Number on FPGA is zero-based
                    
                    status = obj.sendAdapterModuleUserCommand(userCommand,userData0,userData1);
                    assert(status == 0,'Setting DC coupling for channel %d returned fpga error code %d',channelNumber,status);
                end
            end
        end
        
        function set.sampleRateInternal_(obj,val)
           obj.sampleRateInternal_ = val;
           
           % side effect
           obj.hScan.sampleRate = NaN; %fire setter in Scan2D to update UI
        end
        
        
        function val = get.auxTriggersEnable(obj)
           val= obj.hScan.mdfData.auxTriggersEnable; 
        end
        
        function val = get.I2CEnable(obj)
            val= obj.hScan.mdfData.I2CEnable;
        end        
        
        function val = get.I2C_STORE_AS_CHAR(obj)
            val = obj.hScan.mdfData.I2CStoreAsChar;
        end
        
        function val = get.periodsPerFrame(obj)
            linesPerPeriod = 2^(obj.hScan.bidirectional);
            val = obj.linesPerFrame / linesPerPeriod;
        end
        
        function val = get.dcOvervoltage(obj)
            val = obj.hFpga.AcqStatusDCOvervoltage;
        end
        
        function val = get.dataRate(obj)
            pixelsPerRecord = obj.pixelsPerLine * 2^obj.hScan.bidirectional;
            
            if obj.hScan.multiChannel
                bytesPerRecord = pixelsPerRecord * obj.FIFO_ELEMENT_SIZE_BYTES_MULTI_CHAN;
            else
                bytesPerRecord = pixelsPerRecord * obj.FIFO_ELEMENT_SIZE_BYTES_SINGLE_CHAN;
            end
            
            dataRate = bytesPerRecord * obj.hScan.scannerFrequency; % in bytes/second
            val = dataRate / 1E6;   % in MB/s
        end
        
        function val = get.scanFrameRate(obj)
            val = obj.hScan.scannerFrequency*(2^obj.hScan.bidirectional)/(obj.linesPerFrame+obj.flybackLinesPerFrame);
        end
        
        function val = get.fifoSizeFrames(obj)
            % limit size of DMA Fifo
            fifoSizeSecondsLimit = obj.FIFO_SIZE_LIMIT_MB / obj.dataRate;
            fifoSizeSeconds_ = min(obj.FIFO_SIZE_SECONDS,fifoSizeSecondsLimit);
            
            % hold at least 5 frames            
            val = max(5,ceil(obj.scanFrameRate * fifoSizeSeconds_));
        end
        
        function val = get.frameQueueLoggerCapacity(obj)
            % limit size of frame queue
            frameQueueLoggerSizeSecondsLimit = obj.FRAME_QUEUE_LOGGER_SIZE_LIMIT_MB / obj.dataRate;
            frameQueueLoggerSizeSeconds_ = min(obj.FRAME_QUEUE_LOGGER_SIZE_SECONDS,frameQueueLoggerSizeSecondsLimit);
            
            % hold at least 5 frames
            val = max(5,ceil(obj.scanFrameRate * frameQueueLoggerSizeSeconds_));
        end
        
        function val = get.frameQueueMatlabCapacity(obj)
            % limit size of frame queue
            frameQueueMatlabSizeSecondsLimit = obj.FRAME_QUEUE_MATLAB_SIZE_LIMIT_MB / obj.dataRate;
            frameQueueMatlabSizeSeconds_ = min(obj.FRAME_QUEUE_MATLAB_SIZE_SECONDS,frameQueueMatlabSizeSecondsLimit);
            
            % hold at least 3 frames
            val = max(3,ceil(obj.scanFrameRate * frameQueueMatlabSizeSeconds_));

            if obj.SYNC_DISPLAY_TO_VOLUME_RATE && ~isinf(obj.hScan.framesPerStack)
                max(obj.hScan.framesPerStack,0); %make sure this is positive
                
                % queueSizeInVolumes:
                % has to be at least 1
                % the larger this value the longer the delay between acquisition and display
                % the smaller this value, the more volumes might be dropped
                %   in the display, and the display rate might be reduced
                queueSizeInVolumes = 1.5; % 1.5 means relatively small display latency
                
                assert(queueSizeInVolumes >= 1); % sanity check
                val = max(val,ceil(queueSizeInVolumes * obj.hScan.framesPerStack));
            end
        end
        
        function val = get.rawAdcOutput(obj)
            obj.fpgaCheckAdapterModuleInitialization();
            if obj.simulated
                val = [0 0 0 0];
            else
                val = obj.hFpga.DebugRawAdcOutput;
            end
        end

        function val = get.triggerHoldOff(obj)
            val = round(max(0,obj.linePhaseSamples + obj.estimatedPeriodClockDelay));
        end
 
        function val = get.estimatedPeriodClockDelay(obj)
            %TODO: Improve Performance
            totalSamplesPerLine = (obj.hScan.sampleRate / obj.hScan.mdfData.nominalResScanFreq) / 2;
            acqSamplesPerLine = totalSamplesPerLine * obj.hScan.fillFractionTemporal;
            
            val = (totalSamplesPerLine - acqSamplesPerLine)/2;
            
            val = val + obj.ADAPTER_MODULE_TRIGGER_TO_ADC_DELAY(obj.flexRioAdapterModuleName);
            val = round(val);
        end
        
        function val = get.adapterModuleChannelCount(obj)
            val = obj.ADAPTER_MODULE_CHANNEL_COUNT(obj.flexRioAdapterModuleName);
        end
        
        function value = get.beamTiming(obj)      
            sampsPerLine = obj.maskParams.samplesPerLine;
            
            beamOnLeadTicks = round( ( obj.hScan.beamClockDelay / 1e6 ) * obj.hScan.sampleRate );
            beamOffLagTicks = round( ( obj.hScan.beamClockExtend / 1e6 ) * obj.hScan.sampleRate ) - beamOnLeadTicks;
            
            beamClockOnForward  = -beamOnLeadTicks;
            beamClockOffForward = sampsPerLine + beamOffLagTicks;
            
            if obj.hScan.bidirectional && ~obj.hScan.useFpgaBiDir
                sampsPerLineSwitch = obj.maskParams.samplesPerLineSwitch;
                beamClockOnBackward  = sampsPerLine + sampsPerLineSwitch - beamOnLeadTicks;
                beamClockOffBackward = 2 * sampsPerLine + sampsPerLineSwitch + beamOffLagTicks;
            else
                beamClockOnBackward  = 0;
                beamClockOffBackward = 0;
            end
            
            if beamOnLeadTicks > obj.triggerHoldOff
                most.idioms.dispError('Beams switch time is set to precede period clock. This setting cannot be fullfilled.\n');
            end
            
            value = [beamClockOnForward beamClockOffForward beamClockOnBackward beamClockOffBackward];
        end
    end
    
    %System Properties
    methods
        function set.delayMaskComputation(obj,val)            
            obj.delayMaskComputation = val;
            
            %side effect
            if ~obj.delayMaskComputation
                obj.computeMask();
            end
        end
    end
    
    %%% acquisition parameters
    methods
        function val = get.loggingEnable(obj)               % accessed by MEX function
            val = obj.hScan.hSI.hChannels.loggingEnable;
        end
        
        function val = get.channelsActive(obj)              % accessed by MEX function
            val = obj.hScan.hSI.hChannels.channelsActive;
        end
        
        function val = get.channelsDisplay(obj)
            val = obj.hScan.hSI.hChannels.channelDisplay;
            % maybe these checks are a little paranoid, but I really don't want
            % the mex function to crash (GJ)
            val = round(val);
            val = unique(val);
            val(val<1 | val>4) = [];
            val = sort(val);
            val = double(val);
        end
        
        function val = get.channelsSave(obj)
            val = obj.hScan.hSI.hChannels.channelSave;
            % maybe these checks are a little paranoid, but I really don't want
            % the mex function to crash (GJ)
            val = round(val);
            val = unique(val);
            val(val<1 | val>4) = [];
            val = sort(val);
            val = double(val);          
        end
        
        function set.frameTagging(obj,val)
            %validation
            obj.zprpAssertNotRunning('frameTagging');
            validateattributes(val,{'logical' 'numeric'},{'binary' 'scalar'});
            %set prop
            assert(val,'frameTagging cannot be deactivated');
            obj.frameTagging = val;
            %side effects
            obj.flagResizeAcquisition = true;
        end

        function set.frameAcquiredFcn(obj,val)
            %validation
            if isempty(val)
                val = [];
            else
                validateattributes(val,{'function_handle'},{'scalar'});
            end
            
            %set prop
            obj.frameAcquiredFcn = val;
            
            %side effects
            ResonantAcqMex(obj,'registerFrameAcqFcn',val);
        end
    end
    
    %%% live acq params
    methods
        function set.channelsInvert(obj,val)
            validateattributes(val,{'logical','numeric'},{'vector'});
            val = logical(val);
            
            if ~obj.simulated
                if length(val) == 1
                    val = repmat(val,1,obj.adapterModuleChannelCount);
                elseif length(val) < obj.adapterModuleChannelCount
                    val(end+1:end+obj.adapterModuleChannelCount-length(val)) = val(end);
                    most.idioms.warn('ResScan channelsInvert had less entries than physical channels are available. Set to %s',mat2str(val));
                elseif length(val) > obj.adapterModuleChannelCount
                    val = val(1:obj.adapterModuleChannelCount);
                    most.idioms.warn('ResScan channelsInvert had more entries than physical channels are available.');
                end
                
                valFpga = val; % fpga always expects a vector of length 4
                valFpga(end+1:end+4-length(valFpga)) = false;
                obj.hFpga.AcqParamLiveInvertChannels = valFpga;
            end
            
            obj.channelsInvert = val;
        end
        
        function val = get.linePhaseSamples(obj)
            val = obj.hScan.sampleRate * obj.hScan.linePhase;
        end
        
        function set.reverseLineRead(obj,val)
            %validation
            validateattributes(val,{'logical' 'numeric'},{'binary' 'scalar'});
            %set prop
            obj.reverseLineRead = val;
            %side effects
            obj.fpgaUpdateLiveAcquisitionParameters('reverseLineRead');
        end
    end
    
    %Property-access helpers
    methods (Hidden)
        function zprpUpdateMask(obj)
            obj.dispDbgMsg('Sending Mask to FPGA');
            
            % generate the mask write indices and cast the data to the
            % right datatype
            maskWriteIndices = cast(0:(length(obj.mask)-1),'uint16');
            maskData = cast(obj.mask','int16');
            
            % interleave the indices with the mask data and recast it into
            % a uint32. This is the format the MasktoFPGA FIFO expects
            maskToSend = reshape([maskData;maskWriteIndices],1,[]);
            maskToSend = typecast(maskToSend,'uint32');
            
            try
                assert(length(obj.mask) <= 8193,'Length of mask exceeds maximum of 8193 entries'); % Limit of 8193 entries is hard coded on FPGA
                
                %                 Element by element write of mask array to FPGA.
                %                 for i = 1:length(obj.mask)
                %                     obj.hFpga.MaskWriteIndex = i-1;
                %                     obj.hFpga.MaskElementData = obj.mask(i);
                %
                %                     obj.hFpga.MaskDoWriteElement = true;
                %                 end
                
                % Stream Mask to FPGA with a DMA FIFO
                if (~obj.simulated)
                    assert(obj.maskParams.samplesPerPeriod <= intmax(class(obj.hFpga.AcqParamSamplesPerRecord)),...
                        'Too many samples in mask for FPGA to handle'); % sanity check
                    
                    obj.hFpga.fifo_MaskToFPGA.write(maskToSend);
                end
                
                obj.hFpga.AcqParamSamplesPerRecord = obj.maskParams.samplesPerPeriod;
            catch ME
                error('Error sending mask to FPGA device: \n%s',ME.message);
            end
            
            obj.flagUpdateMask = false;
        end
        
        function zprpResizeAcquisition(obj)
            obj.frameSizePixels = obj.pixelsPerLine * obj.linesPerFrame; %not including frame tag
            
            if obj.hScan.multiChannel
                fifoElementSizeBytes = obj.FIFO_ELEMENT_SIZE_BYTES_MULTI_CHAN;
            else
                fifoElementSizeBytes = obj.FIFO_ELEMENT_SIZE_BYTES_SINGLE_CHAN;
            end
            
            obj.tagSizeFifoElements = (obj.FRAME_TAG_SIZE_BYTES / fifoElementSizeBytes) * obj.frameTagging ;
            assert(obj.tagSizeFifoElements == floor(obj.tagSizeFifoElements),'Frame Tag Byte Size must be an integer multiple of FIFO Element Byte Size');
            
            obj.frameSizeFifoElements = obj.frameSizePixels + obj.tagSizeFifoElements;
            obj.frameSizeBytes = obj.frameSizeFifoElements * fifoElementSizeBytes;
            
            if ~obj.simulated
                %Configure FIFO managed by FPGA interface
                if obj.hScan.multiChannel
                    obj.hFpga.fifo_MultiChannelToHostU64.configure(obj.frameSizeFifoElements*obj.fifoSizeFrames);
                    obj.hFpga.fifo_MultiChannelToHostU64.start();
                else
                    obj.hFpga.fifo_SingleChannelToHostI16.configure(obj.frameSizeFifoElements*obj.fifoSizeFrames);
                    obj.hFpga.fifo_SingleChannelToHostI16.start();
                end                
            end
            
            assert((obj.hScan.mdfData.I2CEnable + obj.hScan.mdfData.auxTriggersEnable + obj.hScan.mdfData.photonCountingEnable) <= 1,'I2CEnable, auxTriggersEnable, usePhotonCountinModule are mutually exclusive');
            if obj.hScan.mdfData.I2CEnable
                obj.hFpga.AuxDataFifoMode = 'I2C';
            elseif obj.hScan.mdfData.auxTriggersEnable
                obj.hFpga.AuxDataFifoMode = 'Aux Triggers';
            else
                obj.hFpga.AuxDataFifoMode = 'Disabled';
            end                    
            
            if obj.hScan.mdfData.auxTriggersEnable || obj.hScan.mdfData.I2CEnable
                if ~obj.simulated                   
                    obj.fpgaFifoAuxDataActualDepth = obj.hFpga.fifo_AuxDataToHostU64.configure(obj.FIFO_ELEMENT_SIZE_AUX_DATA);
                    obj.hFpga.fifo_AuxDataToHostU64.start();
                else
                    obj.fpgaFifoAuxDataActualDepth = obj.FIFO_ELEMENT_SIZE_AUX_DATA;
                end 
            end

            
            %Configure queue(s) managed by MEX interface
            ResonantAcqMex(obj,'resizeAcquisition');
            obj.flagResizeAcquisition = false;
        end
    end
    
    
    %% FRIEND METHODS
    
    methods (Hidden, Access = {?scanimage.interfaces.Class})
        
        function start(obj)
            obj.dispDbgMsg('Starting Acquisition');
            
            if ~obj.simulated
                obj.fpgaCheckAdapterModuleInitialization();
                obj.checkAdapterModuleErrorState();
                
                if obj.externalSampleClock
                    obj.measureExternalSampleClockRate();
                end
            end
            
            if obj.dataRate > 200
                most.idioms.dispError('The current acquisition data rate is %.2f MB/s, while the bandwith of PCIe v1.x is 250MB/s. Approaching this limit might result in data loss.\n',obj.dataRate);
            end
            
            if ~obj.simulated && obj.dcOvervoltage
                obj.resetDcOvervoltage();
            end
            
            obj.hFpga.AcqEngineDoReset = true;
            obj.fpgaStartAcquisitionParameters();
            
            obj.delayMaskComputation = false;
            %if obj.flagUpdateMask
                obj.computeMask();
                obj.zprpUpdateMask();
            %end
            
            obj.fpgaSelectFifo();
            
            % reset counters
            obj.framesAcquired = 0;
            obj.acqCounter = 0;
            obj.lastEndOfAcquisition = 0;
            
            obj.zprpResizeAcquisition();
            
            obj.bufferAcqParams();
            
            %Start acquisition
            if ~obj.simulated
                ResonantAcqMex(obj,'syncFpgaClock'); % Sync FPGA clock and system clock
            end
            ResonantAcqMex(obj,'startAcq');     % Arm Frame Copier to receive frames
            obj.hFpga.AcqEngineDoArm = true;    % then start the acquisition
            obj.acqRunning = true;
        end
        
        function abort(obj)
            if ~obj.acqRunning
                return
            end
            
            ResonantAcqMex(obj,'stopAcq');
            
            obj.hFpga.AcqEngineDoReset = true;
            
            if (~obj.simulated)
                obj.fpgaStopFifo();
            end
            
            obj.acqRunning = false;
            
            try
                obj.checkAdapterModuleErrorState();
            catch ME
                most.idioms.reportError(ME);
            end
        end
        
        function stripeData = frameToRois(obj,stripeData,frameData)
            numPlanes = length(obj.acqParamBuffer.zs);
            
            planeNum = mod(stripeData.frameNumberAcqMode-1,obj.acqParamBuffer.framesPerStack)+1;
            
            if planeNum > numPlanes
                % flyback frame
                stripeData.roiData = {};
            else
                z = obj.acqParamBuffer.zs(planeNum);
                stripeData.roiData = {};
                
                if ~isempty(obj.acqParamBuffer.roi)
                    roiData = scanimage.mroi.RoiData();
                    roiData.hRoi = obj.acqParamBuffer.roi;
                    
                    numLines = obj.acqParamBuffer.numLines;
                    startLine = 1;
                    endLine = numLines;
                    
                    roiData.zs = z;
                    roiData.channels = stripeData.channelNumbers;
                    
                    roiData.stripePosition = {[1, numLines]};
                    roiData.stripeFullFrameNumLines = numLines;
                    
                    roiData.frameNumberAcq = stripeData.frameNumberAcq;
                    roiData.frameNumberAcqMode = stripeData.frameNumberAcqMode;
                    roiData.frameTimestamp = stripeData.frameTimestamp;
                    
                    roiData.imageData = cell(length(roiData.channels),1);
                    for chanIdx = 1:length(roiData.channels)
                        if startLine == 1 && endLine == size(frameData{chanIdx},2);
                            % performance improvement for non-mroi mode
                            roiData.imageData{chanIdx}{1} = frameData{chanIdx}; % images are transposed at this point
                        else
                            roiData.imageData{chanIdx}{1} = frameData{chanIdx}(:,startLine:endLine); % images are transposed at this point
                        end
                    end
                    stripeData.roiData{1} = roiData;
                end
            end
        end
        
        function generateSoftwareAcqTrigger(obj)
            obj.hFpga.AcqTriggerDoSoftwareTrig = true;
        end
        
        function generateSoftwareAcqStopTrigger(obj)
            obj.hFpga.StopTriggerDoSoftwareTrig = true;
        end
        
        function generateSoftwareNextFileMarkerTrigger(obj)
            obj.hFpga.AdvanceTriggerDoSoftwareTrig = true;
        end
        
        function signalReadyReceiveData(obj)
            ResonantAcqMex(obj,'signalReadyReceiveData');
        end
        
        function [success,stripeData] = readStripeData(obj)
            stripeData = scanimage.interfaces.StripeData();
            
            if ~obj.acqRunning
                success = false;
                return
            end
            
            % fetch data from Mex function
            [success, frameData, frameTag, framesRemaining, acqStatus] = ResonantAcqMex(obj,'getFrame');
            
            if acqStatus.numDroppedFramesLogger > 0
                try % this try catch is a bit paranoid, but we want to make sure that the acquisition is aborted and some sort of error is printed
                    errorMsg = sprintf('Data logging lags behind acquisition: %d frames lost.\nAcquisition stopped.\n',acqStatus.numDroppedFramesLogger);
                    most.idioms.dispError(errorMsg);
                catch ME
                    most.idioms.reportError(ME);
                end
                
                obj.hScan.hSI.abort();
                
                errordlg(errorMsg,'Error during acquisition','modal');
                success = false;
                return
            end
            
            if ~success
                return % read from empty queue
            end

            if frameTag.frameTagCorrupt
                try % this try catch is a bit paranoid, but we want to make sure that the acquisition is aborted and some sort of error is printed
                    if obj.hScan.multiChannel
                        pixelsLost = obj.hFpga.FifoMultiChannelPixelsLost;
                    else
                        pixelsLost = obj.hFpga.FifoSingleChannelPixelsLost;
                    end

                    fpgaState = obj.hFpga.AcqStatusEngineState; % we have to capture the fpga state before abort() resets it to idle

                    errorMsg = sprintf(['Error: Data of frame %d appears to be corrupt. Acquistion stopped. ',...
                               'Corrupt data was not logged to disk.\n...',...
                               'Most likely pixels were lost because PXIe data bandwidth was exceeded.\n',...
                               'Debug information: %d pixelsLost; Fpga state = %s\n'],...
                                obj.framesAcquired+1,pixelsLost,fpgaState);
                    most.idioms.dispError(errorMsg);
                catch ME
                    most.idioms.reportError(ME);
                end
                
                obj.hScan.hSI.abort();
                
                errordlg(errorMsg,'Error during acquisition','modal');
                success = false;
                return
            end
            
            if frameTag.totalAcquiredFrames == 1
                obj.epochAcqMode = now;
            end
            
            stripeData.endOfAcquisitionMode = frameTag.endOfAcqMode;
            stripeData.endOfAcquisition = frameTag.endOfAcq;
            if strcmp(obj.flexRioAdapterModuleName, 'NI517x')
                stripeData.overvoltage = false;
%                 stripeData.overvoltage = dabs.ni.oscope.checkOverload;
            else
                % need to fix over voltage detection for 517x!!
                stripeData.overvoltage = frameTag.dcOvervoltage;
            end
            stripeData.startOfFrame = true; % there is only one stripe per frame for resonant scanning
            stripeData.endOfFrame = true;   % there is only one stripe per frame for resonant scanning
            stripeData.stripeNumber = 1;    % there is only one stripe per frame for resonant scanning
            stripeData.frameNumberAcqMode = frameTag.totalAcquiredFrames;
            stripeData.frameNumberAcq = frameTag.totalAcquiredFrames - obj.lastEndOfAcquisition;
            stripeData.acqNumber = frameTag.acqNumber;
            stripeData.frameTimestamp = frameTag.frameTimestamp;   
            stripeData.acqStartTriggerTimestamp = frameTag.acqTriggerTimestamp;
            stripeData.nextFileMarkerTimestamp = frameTag.nextFileMarkerTimestamp;
            stripeData.stripesRemaining = framesRemaining;
            stripeData.epochAcqMode = obj.epochAcqMode;
            stripeData.channelNumbers = obj.hScan.hSI.hChannels.channelDisplay; %This is a little slow, since it's dependent: obj.channelsDisplay;
            stripeData = obj.frameToRois(stripeData,frameData); % images are transposed at this point
            
            % update counters
            if stripeData.endOfAcquisition
                obj.acqCounter = obj.acqCounter + 1;
                obj.lastEndOfAcquisition = stripeData.frameNumberAcqMode;
            end
            
            obj.framesAcquired = stripeData.frameNumberAcqMode;
            
            
            % control acquisition
            if stripeData.endOfAcquisitionMode
                obj.abort(); %self-shutdown
            end
        end
    
        function computeMask(obj)
            if obj.delayMaskComputation
                return;
            end
            
            [obj.mask, samplesPerPixel, samplesToSkip] = scanimage.util.computeresscanmask(obj.hScan.scannerFrequency, obj.hScan.sampleRate,...
                 obj.hScan.fillFractionSpatial, obj.pixelsPerLine, obj.hScan.bidirectional && ~obj.hScan.useFpgaBiDir);

            obj.flagUpdateMask = true;
            
            %% side effects
            obj.maskParams.samplesPerPeriod = sum(abs(obj.mask));
            obj.maskParams.samplesPerLine = sum(samplesPerPixel);
            obj.maskParams.samplesPerLineSwitch = samplesToSkip;
            obj.maskParams.scanLineDuration = obj.maskParams.samplesPerLine / obj.hScan.sampleRate;
        end
        
        function configOscopeChannels(obj,newInputRange)
            if nargin > 1 && ~isempty(newInputRange)
                rg = newInputRange;
            elseif isempty(obj.hScan.channelsInputRanges)
                rg = repmat({[0 5]}, 1, 4);
            else
                rg = obj.hScan.channelsInputRanges;
            end
            
            coupling = false; % false = DC coupling; true = AC coupling
            
            for ch = 0:3
                r = rg{ch+1};
                r = r(2) - r(1);
                err = dabs.ni.oscope.configureChannel(ch, r, true, coupling);
                assert(err == 0, 'Error when attempting to configure NI 517x device. Code = %d', err);
            end
        end
        
        function configureAdapterModuleSampleClock(obj)
            obj.hScan.zprvMDFVerify('externalSampleClock',{{'logical'},{'scalar','nonempty'}},[]);
            
            if obj.hScan.mdfData.externalSampleClock;
                configureExternalSampleClock();
            else
                configureInternalSampleClock();
            end
            
            function configureInternalSampleClock()
                if strcmp(obj.flexRioAdapterModuleName, 'NI517x')
                    dabs.ni.oscope.configureSampleClock(false,0);
                else
                    command   = 0; % 0 = Clock Settings
                    userData0 = 3; % 3 = Internal Sample Clock locked to an external Reference Clock through Sync Clock <- FPGA hardcoded to use PXIe_Clk10 as Sync Clock
                    userData1 = 0; % unused
                    
                    status = obj.sendAdapterModuleUserCommand(command,userData0,userData1);
                    assert(status == 0,'Configuring internal sample clock for FlexRio digitizer module failed with status code %d',status);
                end
                
                obj.sampleRateInternal_ = obj.ADAPTER_MODULE_SAMPLING_RATE_MAP(obj.flexRioAdapterModuleName); %cannot update obj.hScan.sampleRate directly
                obj.externalSampleClock = false;
            end
            
            function configureExternalSampleClock()
                fprintf('Setting up external sample clock for FPGA digitizer module.\n');
                obj.externalSampleClock = true; %This needs to be set before the call of sendAdapterModuleUserCommand
                
                assert(~isempty(strfind(obj.flexRioAdapterModuleName,'NI573')),...
                    'External sample clock unsupported for digitizer module %s. Please set the machine data file property ''externalSampleClockRate'' to false and restart ScanImage',...
                    obj.flexRioAdapterModuleName);
                
                obj.hScan.zprvMDFVerify('externalSampleClockRate',{{'numeric'},{'scalar','nonempty','positive'}},[]);
                
                sampleRateRange = obj.ADAPTER_MODULE_SAMPLING_RATE_RANGE_MAP(obj.flexRioAdapterModuleName);
                assert(( min(obj.hScan.mdfData.externalSampleClockRate) >= sampleRateRange(1) ) && ...
                       ( max(obj.hScan.mdfData.externalSampleClockRate) <= sampleRateRange(2) ),...
                    'The sample rate specified in the machine data file ( %.3fMHz ) is outside the supported range of the %s FPGA digitizer module ( %.1f - %.1fMHz )',...
                    obj.hScan.mdfData.externalSampleClockRate/1e6,obj.flexRioAdapterModuleName,sampleRateRange(1)/1e6,sampleRateRange(2)/1e6);
                
                if strcmp(obj.flexRioAdapterModuleName, 'NI517x')
                    dabs.ni.oscope.configureSampleClock(true,obj.hScan.mdfData.externalSampleClockRate);
                else
                    command   = 0; % 0 = Clock Settings
                    userData0 = 2; % 2 = External Sample Clock through the CLK IN connector
                    userData1 = 0; % unused
                    
                    status = obj.sendAdapterModuleUserCommand(command,userData0,userData1);
                    assert(status == 0,'Configuring external sample clock for FlexRio digitizer module failed with status code %d',status);
                end
                
                obj.sampleRateInternal_ = obj.hScan.mdfData.externalSampleClockRate; %preliminary, actual sample rate measured later %cannot update obj.hScan.sampleRate directly
            end
        end
        
        function measureExternalSampleClockRate(obj)
            measurePeriod     = 1e-3;   % [s] count sample clock edges for measurePeriod of time
            numMeasurements   = 100;    % number of measurment repeats to calculate mean and standard deviation
            allowOverClocking = 0.01;   % allow to overclock the digitizer by 1%
            sampleRateRange   = obj.ADAPTER_MODULE_SAMPLING_RATE_RANGE_MAP(obj.flexRioAdapterModuleName);
            maxSampleRateStd  = 3e3;    % [Hz] TODO: need to fine tune this value (somewhat of a guess right now)
            
            obj.checkAdapterModuleErrorState();
            
            % start measuring the sample rate
            fprintf('Measuring FPGA digitizer sample clock frequency...\n');
            obj.hFpga.AcqStatusAcqLoopMeasurePeriod = round(measurePeriod * obj.FPGA_SYS_CLOCK_RATE);
            measurePeriod = double(obj.hFpga.AcqStatusAcqLoopMeasurePeriod) / obj.FPGA_SYS_CLOCK_RATE; %read measure period back to account for rounding errors
            
            measurements = zeros(numMeasurements,1);
            for iter = 1:numMeasurements
                measurements(iter) = obj.hFpga.AcqStatusAcqLoopIterationsCount / measurePeriod;
                most.idioms.pauseTight(measurePeriod);
            end
            
            sampleRateMean = mean(measurements);
            sampleRateStd  = std(measurements);
            
            if sampleRateMean < 1e3
               most.idioms.dispError(['The external sample rate frequency %.1fHz is suspiciously low. ',...
                   'Is the clock connected and running?\n\n'],sampleRateMean) ;
            end
            
            if ( sampleRateMean < sampleRateRange(1)*(1-allowOverClocking) ) || ...
               ( sampleRateMean > sampleRateRange(2)*(1+allowOverClocking) )
                
               plotMeasurement(measurements);
               error('The external sample clock frequency %.3fMHz is outside the supported range of the %s FPGA digitizer module (%.1f - %.1fMHz).',...
                      sampleRateMean/1e6,obj.flexRioAdapterModuleName,sampleRateRange(1)/1e6,sampleRateRange(2)/1e6);
            end
               
            if sampleRateStd > maxSampleRateStd
                plotMeasurement(measurements);
                error('The external sample clock of the FPGA digitizer module is unstable. Sample frequency mean: %.3EHz, SD: %.3EHz. Please make sure the sample clock is connected and running.',...
                    sampleRateMean,sampleRateStd); % GJ 2015-03-01 <- if this check fails, we might have to adjust maxSampleRateStd
            end
            
            %if all checks passed, save sample rate to property            
            obj.sampleRateInternal_ = sampleRateMean;
            fprintf('FPGA digitizer module external sample clock is stable at %.3fMHz (SD: %.0fHz)\n',sampleRateMean/1e6,sampleRateStd);
            
            %local function
            function plotMeasurement(measurements)
                persistent hFig
                if isempty(hFig) || ~ishghandle(hFig)
                    hFig = figure('Name','FPGA digitizer module sample frequency','NumberTitle','off','MenuBar','none');
                end
                
                clf(hFig);
                figure(hFig); %bring to front
                
                hAx = axes('Parent',hFig);
                plot(hAx,linspace(1,measurePeriod * numMeasurements,numMeasurements),measurements);
                title(hAx,'FPGA digitizer module sample frequency');
                xlabel(hAx,'Time [s]');
                ylabel(hAx,'Sample Frequency [Hz]');
            end
        end
        
        function checkAdapterModuleErrorState(obj)
            if ~obj.simulated
                sampleRateRange = obj.ADAPTER_MODULE_SAMPLING_RATE_RANGE_MAP(obj.flexRioAdapterModuleName);
                assert(obj.hFpga.AdapterModuleUserError == 0,...
                    'Fatal error: The FlexRio adapter module became instable and needs to be reset. Please restart ScanImage to reinitialize the module. If you use an external sample clock, do not disconnect the clock while ScanImage is running and ensure the clock rate is within the range %.1E - %.1E Hz',...
                    sampleRateRange(1),sampleRateRange(2));
            end
        end
        
        function status = sendAdapterModuleUserCommand(obj,userCommand,userData0,userData1)
            if obj.simulated
                status = 0;
                return
            end
            
            if isempty(strfind(obj.flexRioAdapterModuleName,'NI573'))
                obj.dispDbgMsg('Adapter module %s does not support user commands',obj.flexRioAdapterModuleName);
                status = 0;
                return
            end
            
            obj.fpgaCheckAdapterModuleInitialization();
            obj.checkAdapterModuleErrorState();
            
            % Wait for module to be ready to accept user command input
            checkModuleIdle();
            
            % Execute user command
            obj.hFpga.AdapterModuleUserCommand = userCommand;
            obj.hFpga.AdapterModuleUserData0 = userData0;
            obj.hFpga.AdapterModuleUserData1 = userData1;
            obj.hFpga.AdapterModuleDoUserCommandCommit = true;
            obj.AMCmdSent = false;
            
            % Check user command return value
            checkModuleIdle();
            status = obj.hFpga.AdapterModuleUserCommandStatus;
            
            % nested function
            function checkModuleIdle()
                moduleIsIdle = obj.waitModuleUserCommandIdle;
                if ~moduleIsIdle
                    if obj.externalSampleClock
                        most.idioms.dispError(['Sending a user command to the FPGA failed. ',...
                            'This can be caused by an unstable external sample clock.\n']);
                        obj.measureExternalSampleClockRate;
                    end
                    assert(obj.waitModuleUserCommandIdle,'Module is not idle - failed to send command');
                end
            end
        end
        
        function sendNonBlockingAdapterModuleUserCommand(obj,userCommand,userData0,userData1)
            if obj.simulated
                return
            end
            
            if isempty(strfind(obj.flexRioAdapterModuleName,'NI573'))
                obj.dispDbgMsg('Adapter module %s does not support user commands',obj.flexRioAdapterModuleName);
                return
            end
            
            obj.fpgaCheckAdapterModuleInitialization();
            obj.checkAdapterModuleErrorState();
            
            % Wait for module to be ready to accept user command input
            assert(obj.waitModuleUserCommandIdle,'Module is not idle - failed to send command');
            
            % Execute user command
            obj.hFpga.AdapterModuleUserCommand = userCommand;
            obj.hFpga.AdapterModuleUserData0 = userData0;
            obj.hFpga.AdapterModuleUserData1 = userData1;
            obj.hFpga.AdapterModuleDoUserCommandCommit = true;
            obj.AMCmdSent = true;
        end
        
        function fpgaUpdateLiveAcquisitionParameters(obj,property)
            if obj.acqRunning || strcmp(property,'forceall')
                obj.dispDbgMsg('Updating FPGA Live Acquisition Parameter: %s',property);
                
                if updateProp('linePhaseSamples')
                    preTrigSamps =  obj.linePhaseSamples + obj.estimatedPeriodClockDelay;                                        
                    
                    if preTrigSamps < 0
                        preTrigSamps = abs(preTrigSamps);
                    else
                        preTrigSamps = 0;
                    end                    
                    
                    obj.hFpga.AcqParamLiveTriggerHoldOff = uint32(obj.triggerHoldOff);
                    obj.hFpga.AcqParamLivePreTriggerSamples = uint16(preTrigSamps);
                end
                
                if updateProp('beamClockDelay')
                    beamTiming = obj.beamTiming;
                    obj.hFpga.BeamClockOnForward   = beamTiming(1);
                    obj.hFpga.BeamClockOffForward  = beamTiming(2);
                    obj.hFpga.BeamClockOnBackward  = beamTiming(3);
                    obj.hFpga.BeamClockOffBackward = beamTiming(4);
                end
                
                if updateProp('reverseLineRead')
                    obj.hFpga.AcqParamLiveReverseLineRead = obj.reverseLineRead;
                end
            end
            
            % Helper function to identify which properties to update
            function tf = updateProp(currentprop)
                tf = strcmp(property,'forceall') || strcmp(property,currentprop);
            end
        end
        
        function bufferAcqParams(obj,live)
            if nargin < 2 || isempty(live) || ~live
                obj.acqParamBuffer = struct(); % flush buffer
            end
            
            roiGroup = obj.hScan.currentRoiGroupScannerCoords;
            zs=obj.hScan.hSI.hStackManager.zs; % generate planes to scan based on motor position etc

            obj.acqParamBuffer.roi = roiGroup.rois;
            obj.acqParamBuffer.numLines = roiGroup.rois.scanfields.pixelResolution(2);
            
            if nargin < 2 || isempty(live) || ~live
                obj.acqParamBuffer.zs = zs;
                obj.acqParamBuffer.numPlanes  = numel(zs);
                obj.acqParamBuffer.framesPerStack = obj.acqParamBuffer.numPlanes + obj.hScan.hSI.hFastZ.numDiscardFlybackFrames;
            end
            
        end
    end
    
    %% INTERNAL METHODS
    methods (Access = private)        
        function fpgaStartAcquisitionParameters(obj)
            obj.dispDbgMsg('Initializing Acquisition Parameters on FPGA');
                        
            if isinf(obj.hScan.framesPerStack)
                framesPerStack_ = 0;
            else
                framesPerStack_ = obj.hScan.framesPerStack;
            end

            if isinf(obj.hScan.framesPerAcq)
                framesPerAcquisition_ = 0;
            else
                framesPerAcquisition_ = obj.hScan.framesPerAcq;
            end
            
            if isinf(obj.hScan.trigAcqNumRepeats)
                acquisitionsPerAcquisitionMode_ = 0;
            else
                acquisitionsPerAcquisitionMode_ = obj.hScan.trigAcqNumRepeats;
            end
            
            if obj.hScan.useFpgaBiDir
                obj.hFpga.AcqParamRecordsPerFrame = obj.linesPerFrame;
                obj.hFpga.BidirectionalAcq = obj.hScan.bidirectional;
            else
                obj.hFpga.AcqParamRecordsPerFrame = obj.periodsPerFrame;
                obj.hFpga.BidirectionalAcq = false;
            end
            obj.hFpga.AcqParamFramesPerGrab = framesPerAcquisition_;
            obj.hFpga.AcqParamFramesPerStack = framesPerStack_;
            obj.hFpga.AcqParamGrabsPerAcquisition = acquisitionsPerAcquisitionMode_;
            obj.hFpga.AcqParamFlybackPeriods = obj.flybackLinesPerFrame / 2^obj.hScan.bidirectional;
            obj.hFpga.AcqParamFrameTaggingEnable = obj.frameTagging;
            obj.hFpga.DebugProduceDummyData = obj.dummyData;
            obj.hFpga.PeriodClockDebounce = ceil(obj.sampleRateInternal_ * obj.hScan.mdfData.PeriodClockDebounceTime) * logical(~obj.dummyData);
            obj.hFpga.TriggerDebounce = ceil(obj.sampleRateInternal_ * obj.hScan.mdfData.TriggerDebounceTime);
            
            % valid values for obj.hFpga.AdvanceTriggerType and obj.hFpga.StopTriggerType:
            % {'External','External or Software','Immediate','None','Software'}
            if obj.hScan.trigNextStopEnable
                obj.hFpga.AdvanceTriggerType = 'External or Software';
                obj.hFpga.StopTriggerType = 'External or Software';
            else
                obj.hFpga.AdvanceTriggerType = 'None';
                obj.hFpga.StopTriggerType = 'None';
            end
            
            % Configure Trigger Lines
            hTrig = obj.hScan.hTrig;
            obj.hFpga.PeriodClockTerminalIn = hTrig.getPXITerminal('periodClockIn');
            
            if strcmp(obj.hScan.hTrig.digitalIODeviceType, 'FPGA')
                obj.hFpga.AcqTriggerTerminalIn = obj.hScan.trigAcqInTerm;
                obj.hFpga.AdvanceTriggerTerminalIn = obj.hScan.trigNextInTerm;
                obj.hFpga.StopTriggerTerminalIn = obj.hScan.trigStopInTerm;
            else
                obj.hFpga.AcqTriggerTerminalIn = hTrig.getPXITerminal('acqTriggerIn');
                obj.hFpga.AdvanceTriggerTerminalIn = hTrig.getPXITerminal('nextFileMarkerIn');
                obj.hFpga.StopTriggerTerminalIn = hTrig.getPXITerminal('acqStopTriggerIn');
            end
            
            obj.hFpga.PeriodClockOnFallingEdge = hTrig.periodClockOnFallingEdge;
            obj.hFpga.AcqTriggerOnFallingEdge = hTrig.acqTriggerOnFallingEdge;
            obj.hFpga.AdvanceTriggerOnFallingEdge = hTrig.nextFileMarkerOnFallingEdge;
            obj.hFpga.StopTriggerOnFallingEdge = hTrig.acqStopTriggerOnFallingEdge;
            
            %TODO: This never changes right? If so, we should set outside of this start() helper fcn
            obj.hFpga.FrameClockTerminalOut = hTrig.getPXITerminal('frameClockOut'); 
            obj.hFpga.BeamClockTerminalOut =  hTrig.getPXITerminal('beamModifiedLineClockOut');
            obj.hFpga.AcqTriggerTerminalOut = hTrig.getPXITerminal('acqTriggerOut');
            
            %set up resonant timebase
            obj.hFpga.MaxResonantPeriodTicks = floor(obj.hScan.nomResPeriodTicks*1.1);
            obj.hFpga.MinResonantPeriodTicks = floor(obj.hScan.nomResPeriodTicks*0.9);
            obj.hFpga.SettlingPeriods = obj.hScan.resonantSettlingPeriods;
            obj.hFpga.ResonantTimeBasePulsesPerPeriod = obj.hScan.resonantTimebaseTicksPerPeriod;
            if obj.hScan.useResonantTimebase
                obj.hFpga.BeamClockTerminalOut =  '';
                obj.hFpga.ResonantTimebaseTerminalOut = hTrig.getPXITerminal('beamModifiedLineClockOut');
            end
            
            if ~isempty(obj.channelsActive)
                obj.hFpga.AcqParamLiveSelectSingleChannel = obj.channelsActive(1) - 1;
            end
            
            %From Georg:
            %BeamClockMode values: 'Trigger' or 'Toggle'
            %
            %Trigger: the clock signal is high for the duration of the beam
            %Toggle:  the clock signal generates a 75ns pulse when the beam changes its on/off status
            obj.hFpga.BeamClockMode = 'Trigger';
            
            if obj.hScan.mdfData.photonCountingEnable
                obj.hFpga.PhotonCountingEnable = true;
                
                obj.hFpga.PhotonCountingDebounce = round(obj.hScan.mdfData.photonCountingDebounce * obj.sampleRateInternal_);
                obj.hFpga.PhotonCountScale = obj.hScan.mdfData.photonCountingScaleByPowerOfTwo; 
                
                obj.hScan.mdfData.auxTriggersEnable = false; % because FPGA PFI0..3 is used for photon counting
                obj.hScan.mdfData.I2CEnable = false; % because FPGA PFI0..3 is used for photon counting
                obj.hFpga.MaskDisableAveraging = obj.hScan.mdfData.photonCountingDisableAveraging;
            else
                obj.hFpga.PhotonCountingEnable = false;
                obj.hFpga.MaskDisableAveraging = false;
            end
            
            % configure aux triggers
            obj.hFpga.AuxTriggerEnable = obj.hScan.mdfData.auxTriggersEnable;
            obj.hFpga.AuxTriggerDebounce = max(round(obj.hScan.mdfData.auxTriggersTimeDebounce * obj.FPGA_SYS_CLOCK_RATE),1); % could be a live prop
            obj.hFpga.AuxTriggerInvertLines = obj.hScan.mdfData.auxTriggerLinesInvert; % could be a live prop
            
            % configure I2C engine
            obj.hFpga.I2CEnable = obj.hScan.mdfData.I2CEnable;
            obj.hFpga.I2CAddress = obj.hScan.mdfData.I2CAddress;
            obj.hFpga.I2CDebounce = round(obj.hScan.mdfData.I2CDebounce * obj.sampleRateInternal_);
            obj.hFpga.I2CDisableACKOutput = obj.hScan.mdfData.I2CDisableAckOutput;
            
            %additionally update the Live Acquisition Parameters
            if (~obj.simulated)
                obj.fpgaUpdateLiveAcquisitionParameters('forceall');
            end
        end
        
        function fpgaReset(obj)
            obj.dispDbgMsg('Resetting FPGA');
            if (~obj.simulated)
                obj.hFpga.reset();
                obj.hFpga.run();
            end
            obj.dispDbgMsg('Resetting FPGA completed');
        end
        
        function fpgaCheckAdapterModuleInitialization(obj)
            obj.dispDbgMsg('checking FPGA Adapter Module Initialization');
            timeout = obj.HW_TIMEOUT;           %timeout in seconds
            pollinginterval = obj.HW_DETECT_POLLING_INTERVAL; %pollinginterval in seconds
            while obj.hFpga.AdapterModuleInitializationDone == 0
                pause(pollinginterval);
                timeout = timeout - pollinginterval;
                if timeout <= 0
                    error('Initialization of adapter module timed out')
                end
            end
            obj.dispDbgMsg('FPGA Adapter Module is initialized');
        end
        
        function fpgaModuleName = fpgaDetectAdapterModule(obj)
            obj.dispDbgMsg('Detecting FlexRIO Adapter Module');
            
            if strncmp(obj.hScan.mdfData.fpgaModuleType, 'NI517', 5)
                fpgaModuleName = 'NI517x';
                return;
            elseif obj.simulated
                % fake adapter module
                if strfind(obj.hScan.mdfData.digitizerModuleType,'5732')
                    expectedModuleID = 278099318;
                    insertedModuleID = 278099318;
                elseif strfind(obj.hScan.mdfData.digitizerModuleType,'5733')
                    expectedModuleID = 278099319;
                    insertedModuleID = 278099319;
                else
                    % fake a NI5734 adapter module
                    expectedModuleID = 278099349;
                    insertedModuleID = 278099349;
                end
            else
                startPolling = tic;
                while obj.hFpga.AdapterModulePresent == 0 || obj.hFpga.AdapterModuleIDInserted == 0
                    pause(obj.HW_DETECT_POLLING_INTERVAL);
                    if toc(startPolling) > obj.HW_DETECT_TIMEOUT
                        error('No FlexRIO Adapter Module installed');
                    end
                end
                
                % get the adapter module name
                expectedModuleID = obj.hFpga.AdapterModuleIDExpected;
                insertedModuleID = obj.hFpga.AdapterModuleIDInserted;
            end
            
            expectedModuleName = obj.ADAPTER_MODULE_MAP(expectedModuleID);
            if isKey(obj.ADAPTER_MODULE_MAP,insertedModuleID)
                fpgaModuleName = obj.ADAPTER_MODULE_MAP(insertedModuleID);
            else
                fpgaModuleName = sprintf('Unknown Module ID: %d', insertedModuleID);
            end

            if ~obj.simulated
                %check if right module is installed
                assert(obj.hFpga.AdapterModuleIDMismatch == 0,...
                    'Wrong Adapter Module installed. Expected Module: ''%s'', Inserted Module:''%s''',...
                    expectedModuleName,fpgaModuleName);
            end
            
            %Display debug info            
            obj.dispDbgMsg('FlexRIO Adapter Module detected: % s',fpgaModuleName);
            obj.dispDbgMsg('FlexRIO Acquisition Sampling Rate: % dHz', obj.ADAPTER_MODULE_SAMPLING_RATE_MAP(fpgaModuleName))
            obj.dispDbgMsg('FlexRIO Channel Count: %d',obj.ADAPTER_MODULE_CHANNEL_COUNT(fpgaModuleName));
            obj.dispDbgMsg('FlexRIO Channel Resolution: %d bits',obj.ADAPTER_MODULE_ADC_BIT_DEPTH(fpgaModuleName));
        end
        
        function fpgaSelectFifo(obj)
            obj.hFpga.FifoEnableSingleChannel = ~obj.hScan.multiChannel;
            obj.hFpga.FifoEnableMultiChannel = obj.hScan.multiChannel;
        end
        
        function fpgaStopFifo(obj)
            obj.dispDbgMsg('Stopping FIFO');
            if obj.hScan.multiChannel
                flushFifo(obj.hFpga.fifo_MultiChannelToHostU64);
                obj.hFpga.fifo_MultiChannelToHostU64.stop();
            else
                flushFifo(obj.hFpga.fifo_SingleChannelToHostI16);
                obj.hFpga.fifo_SingleChannelToHostI16.stop();
            end
            
            flushFifo(obj.hFpga.fifo_AuxDataToHostU64);
            obj.hFpga.fifo_AuxDataToHostU64.stop();
            
            function flushFifo(fifo)
                timeout = 5;
                starttime = tic;
                elremaining = 1;
                while elremaining > 0
                    try
                        [~,elremaining] = fifo.read(elremaining,0);
                    catch ME
                        if ~isempty(strfind(ME.message,'-50400')); % filter timeout error
                            break
                        end
                        most.idioms.reportError(ME);
                    end
                    if toc(starttime) >= timeout
                        most.idioms.warn('Could not flush fifo %s within timeout.',fifo.fifoName);
                        break
                    end
                end
            end
        end
		
        function idle = waitModuleUserCommandIdle(obj)
            % Wait for FPGA to be ready to accept user command inputs
            idle = true;
            start = tic();
            while obj.hFpga.AdapterModuleUserCommandIdle == 0
                if toc(start) > obj.HW_TIMEOUT
                    idle = false;
                    return;
                else
                    pause(obj.HW_POLLING_INTERVAL);
                end
            end
            
            status = obj.hFpga.AdapterModuleUserCommandStatus;
            if status && obj.AMCmdSent
                cmd = int2str(obj.hFpga.AdapterModuleUserCommand);
                most.idioms.warn(['Previous FPGA adapter module command (''' cmd ''') failed with status code ''' int2str(status) '''.']);
            end
        end
    end
    
    %% Private Methods for Debugging
    methods (Access = private)
        function dispDbgMsg(obj,varargin)
            if obj.debugOutput
                fprintf(horzcat('Class: ',class(obj),': ',varargin{1},'\n'),varargin{2:end});
            end
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
