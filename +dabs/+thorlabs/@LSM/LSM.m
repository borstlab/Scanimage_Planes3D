classdef LSM < dabs.thorlabs.private.ThorDevice
    %LSM Class encapsulating Laser Scanning Microscopy Devices offered by Thorlabs
    
    
    %% NOTES
    %
    %   TODO: (VI062511) Add framePeriodEstimate dependent property which estimates frame period at current settings
    %   TODO: Currently setLoggingProperty() allows changes to loggingHeaderString & loggingFileName during live acquisition (though not recommended). However, this is problematic for loggingHeaderString -- a change to just that property would start the current file over wt
    %   TODO: Use some parsing scheme to create and fill-in values for class-added properties listing out options for triggerMode (e.g. 'triggerModes'), and other enumerated properties.
    
    %   TODO: Move accessDeviceCheckoutList to ThorDevice (centralized store of Dabs.Devices)
    
    %% ABSTRACT PROPERTY REALIZATIONS (dabs.thorlabs.private.ThorDevice)
    properties (Constant, Hidden)
        deviceTypeDescriptorSDK = 'Camera'; %Descriptor used by SDK for device type in function calls, e.g. 'Device', 'Camera', etc.
        prop2ParamMap=zlclInitProp2ParamMap(); %Map of class property names to API-defined parameters names
    end
    
    
    %% ABSTRACT PROPERTY REALIZATIONS (most.APIWrapper)
    
    %Following MUST be supplied with non-empty values for each concrete subclass
    properties (Constant, Hidden)
        apiPrettyName='Thorlabs LSM';  %A unique descriptive string of the API being wrapped
        apiCompactName='ThorlabsLSM'; %A unique, compact string of the API being wrapped (must not contain spaces)
        
        %Properties which can be indexed by version
        apiDLLNames = 'ThorConfocal'; %Either a single name of the DLL filename (sans the '.dll' extension), or a Map of such names keyed by values in 'apiSupportedVersionNames'
        %apiHeaderFilenames = { 'LSM_SDK_MOD.h' 'LSM_SDK_MOD.h' 'LSM_SDK_MOD.h' 'LSM_SDK.h' 'LSM_SDK.h' 'LSM_SDK.h' 'LSM_SDK.h' 'LSM_SDK.h'}; %Either a single name of the header filename (with the '.h' extension - OR a .m or .p extension), or a Map of such names keyed by values in 'apiSupportedVersionNames'
        apiHeaderFilenames = 'ThorConfocal_proto.m';
        
    end
    
    
    %% VISIBLE PROPERTIES
    
    %PDEP properties corresponding directly to 'params' defined by API
    properties (SetObservable, GetObservable)
        triggerMode; %One of {'SW_SINGLE_FRAME', 'SW_MULTI_FRAME', 'SW_FREE_RUN_MODE', 'HW_SINGLE_FRAME', 'HW_MULTI_FRAME_TRIGGER_FIRST'}
        triggerTimeout=inf; %Time, in seconds, within which external start trigger is expected to arrive
        triggerFrameClockWithExtTrigger=true; %<Logical>If true, frame clock signal is generated when external (hardware) triggering is enabled. This adds some latency...
        multiFrameCount; %Number of frames to acquire when using triggerMode='SW_MULTI_FRAME' or 'HW_MULTI_FRAME_TRIGGER_FIRST'
        
        pixelsPerLine; %Number of pixels per line
        linesPerFrame; %Number of lines per frame
        fieldSize; %Value from 1-255 setting the field-size
        aspectRatioY; %Value which scales Y amplitude relative to fast X dimension amplitude. Value of 100 matches X dimension amplitude.
        areaMode; %One of {'SQUARE', 'RECTANGLE', 'LINE'}
        offsetX;
        offsetY;
        
        scanMode; %One of {'TWO_WAY_SCAN', 'FORWARD_SCAN', 'BACKWARD_SCAN'}
        bidiPhaseAlignment; %Value from -127-128 allowing bidi scan adjustment ('TWO_WAY_SCAN' mode)
        
        averagingMode; %One of {'AVG_NONE', 'AVG_CUMULATIVE'};
        averagingNumFrames; %Number of frames to average, when averagingMode = 'AVG_CUMULATIVE'
        dataMappingMode; %One of {'POLARITY_INDEPENDENT' 'POLARITY_POSITIVE' 'POLARITY_NEGATIVE'}
        
        inputChannelRange1; %One of the valid input ranges (e.g. 'INPUT_RANGE_20_MV')
        inputChannelRange2; %One of the valid input ranges (e.g. 'INPUT_RANGE_20_MV')
        inputChannelRange3; %One of the valid input ranges (e.g. 'INPUT_RANGE_20_MV')
        inputChannelRange4; %One of the valid input ranges (e.g. 'INPUT_RANGE_20_MV')
        
        clockSource=1; %<1=Internal, 2=External> Specifies clock source for synchronizing to laser pulse train rate
        clockRate=80e6; %Specify clock rate correpsonding to laser pulse train
        
        flybackTimeLines;
        galvoEnable;  %Logical. If true, galvo (Y) mirror is scanned in synchrony with fast (X) resonant-scanned mirror. Setting to false allow line-scanning or independent control of the galvo Y mirror.
        
        
        captureWithoutScanner; %Logical. When true, acquisition occur without scanner being activated. Useful for measuring input voltage soffset values.
        
    end
    
    %PDEP properties created by this class
    properties (GetObservable,SetObservable)
        frameCount;          % Total number of frames that have been acquired by LSM (including those that may have been dropped during processing)
        framesAvailable;     % Number of frames currently available to to read from processed data queue, via getData()
        
        circBufferSize=4;  % size of the circular buffer in frames
        
        loggingFileName=''; %Full filename of logging file (but without path)
        loggingAveragingFactor=1; %Number of frames to average before writing to disk (decimating data stream)
        %         loggingFilePath;
        %         loggingFileName='lsm_data';
        %         loggingFileType='tif';  %One of {'tif' 'bin'} %TODO: Actually use this -- or eliminate it!
        
        frameEventDecimationFactor=1; %Decimation factor to use when generating frame acquired events
        
        channelsLogging; %Logical array of channels, of length numChannelsAvailable, indicating which are designated for logging to disk (when logging is enabled)
        channelsViewing; %Logical array of channels, of length numChannelsAvailable, indicating which are designated for access ('viewing') via getData() methods
        
        subtractChannelOffsets; %Logical array of N values, for each of the N numChannelsAvailable. True values indicate that last-measured offset value for that channel will be subtracted from input data, affecting both logged data and frames returned by getData()
        channelOffsets; %Array of N integer values contaning the last-measured offset values for each of the N numChannelsAvailable
    end
    
    
    properties
        loggingAutoStart=false; %Flag specifying whether to automatically start logging on start()
        
        frameAcquiredEventFcn; %Function handle
        restartOnParamChange = true; %Logical. If true, any active scan is stopped/restarted on changes to an underlying LSM parameter defined by Thor API.  Setting to false allows property (parameter) changes to be batched up without multiple restarts. The method startAlreadyRunning() can be used to stop/restart an ongoing scan, as needed following such a batch of property/param changes.
    end
    
    %Dependent properties part of public API
    properties (Dependent)
        bidiPhaseAlignmentCoarse; %Value from -127-128, pertaining to current fieldSize value, allowing bidi scan adjustment ('TWO_WAY_SCAN' mode). Value is added to bidiPhaseAlignment.
        loggingFrameDelayMax; %Max value of loggingFrameDelay allowed at current settings
        signedData; %Logical indicating whether image data is signed (true) or unsigned (false)
    end
    
    %Read-only
    properties (SetAccess=protected)
        framesGotten; %Number of frames retrieved since start of acquisition via getData() method
        
        state = 'idle'; %One of {'idle' 'armed' 'active' 'pointing'}
        logging = false; %Flag indicating whether file logging is currently occurring
    end
    
    
    %Constructor-initialized, read-only
    properties (SetAccess=protected)
        hPMTModule; %PMT module which /must/ be loaded for successful scanner operation
        numChannelsAvailable; %Number of input channels available for this scanner device
    end
    
    %% HIDDEN PROPERTIES
    
    %Hidden PDEP properties corresponding directly to 'params' defined by API
    properties (GetObservable,SetObservable,Hidden)
        channelsActive; %Array identifying which channels are active, e.g. 1, [1 2], etc.
        touchParameter; %Parameter that can be set to force Thor API to recognize a changed parameter on SetupAcquisition call, thereby restarting acq thread on StartAcquisition() call
        
        bidiPhaseAlignmentCoarse1; %bidiPhaseAlignmentCoarse value for fieldSize 255
        bidiPhaseAlignmentCoarse2; %bidiPhaseAlignmentCoarse value for fieldSize 254
        bidiPhaseAlignmentCoarse3; %bidiPhaseAlignmentCoarse value for fieldSize 253
        bidiPhaseAlignmentCoarse4; %bidiPhaseAlignmentCoarse value for fieldSize 51-65
        bidiPhaseAlignmentCoarse5; %bidiPhaseAlignmentCoarse value for fieldSize 66-80
        bidiPhaseAlignmentCoarse6; %bidiPhaseAlignmentCoarse value for fieldSize 81-95
        bidiPhaseAlignmentCoarse7; %bidiPhaseAlignmentCoarse value for fieldSize 96-110
        bidiPhaseAlignmentCoarse8; %bidiPhaseAlignmentCoarse value for fieldSize 111-125
        bidiPhaseAlignmentCoarse9; %bidiPhaseAlignmentCoarse value for fieldSize 126-140
        bidiPhaseAlignmentCoarse10; %bidiPhaseAlignmentCoarse value for fieldSize 141-155
        bidiPhaseAlignmentCoarse11; %bidiPhaseAlignmentCoarse value for fieldSize 156-170
        bidiPhaseAlignmentCoarse12; %bidiPhaseAlignmentCoarse value for fieldSize 171-185
        bidiPhaseAlignmentCoarse13; %bidiPhaseAlignmentCoarse value for fieldSize 186-200
        bidiPhaseAlignmentCoarse14; %bidiPhaseAlignmentCoarse value for fieldSize 201-215
        bidiPhaseAlignmentCoarse15; %bidiPhaseAlignmentCoarse value for fieldSize 216-230
        bidiPhaseAlignmentCoarse16; %bidiPhaseAlignmentCoarse value for fieldSize 231-255
        bidiPhaseAlignmentCoarse17
        bidiPhaseAlignmentCoarse18
        bidiPhaseAlignmentCoarse19;
        bidiPhaseAlignmentCoarse20;
        bidiPhaseAlignmentCoarse21;
        bidiPhaseAlignmentCoarse22;
        bidiPhaseAlignmentCoarse23;
        bidiPhaseAlignmentCoarse24;
        bidiPhaseAlignmentCoarse25;
        bidiPhaseAlignmentCoarse26;
        bidiPhaseAlignmentCoarse27;
        bidiPhaseAlignmentCoarse28;
        bidiPhaseAlignmentCoarse29;
        bidiPhaseAlignmentCoarse30;
        bidiPhaseAlignmentCoarse31;
        bidiPhaseAlignmentCoarse32;
        bidiPhaseAlignmentCoarse33;
        bidiPhaseAlignmentCoarse34;
        bidiPhaseAlignmentCoarse35;
        bidiPhaseAlignmentCoarse36;
        bidiPhaseAlignmentCoarse37;
        bidiPhaseAlignmentCoarse38;
        bidiPhaseAlignmentCoarse39;
        bidiPhaseAlignmentCoarse40;
        bidiPhaseAlignmentCoarse41;
        bidiPhaseAlignmentCoarse42;
        bidiPhaseAlignmentCoarse43;
        bidiPhaseAlignmentCoarse44;
        bidiPhaseAlignmentCoarse45;
        bidiPhaseAlignmentCoarse46;
        bidiPhaseAlignmentCoarse47;
        bidiPhaseAlignmentCoarse48;
        bidiPhaseAlignmentCoarse49;
        bidiPhaseAlignmentCoarse50;
        bidiPhaseAlignmentCoarse51;
        bidiPhaseAlignmentCoarse52;
        bidiPhaseAlignmentCoarse53;
        bidiPhaseAlignmentCoarse54;
        bidiPhaseAlignmentCoarse55;
        bidiPhaseAlignmentCoarse56;
        bidiPhaseAlignmentCoarse57;
        bidiPhaseAlignmentCoarse58;
        bidiPhaseAlignmentCoarse59;
        bidiPhaseAlignmentCoarse60;
        bidiPhaseAlignmentCoarse61;
        bidiPhaseAlignmentCoarse62;
        bidiPhaseAlignmentCoarse63;
        bidiPhaseAlignmentCoarse64;
        bidiPhaseAlignmentCoarse65;
        bidiPhaseAlignmentCoarse66;
        bidiPhaseAlignmentCoarse67;
        bidiPhaseAlignmentCoarse68;
        bidiPhaseAlignmentCoarse69;
        bidiPhaseAlignmentCoarse70;
        bidiPhaseAlignmentCoarse71;
        bidiPhaseAlignmentCoarse72;
        bidiPhaseAlignmentCoarse73;
        bidiPhaseAlignmentCoarse74;
        bidiPhaseAlignmentCoarse75;
        bidiPhaseAlignmentCoarse76;
        bidiPhaseAlignmentCoarse77;
        bidiPhaseAlignmentCoarse78;
        bidiPhaseAlignmentCoarse79;
        bidiPhaseAlignmentCoarse80;
        bidiPhaseAlignmentCoarse81;
        bidiPhaseAlignmentCoarse82;
        bidiPhaseAlignmentCoarse83;
        bidiPhaseAlignmentCoarse84;
        bidiPhaseAlignmentCoarse85;
        bidiPhaseAlignmentCoarse86;
        bidiPhaseAlignmentCoarse87;
        bidiPhaseAlignmentCoarse88;
        bidiPhaseAlignmentCoarse89;
        bidiPhaseAlignmentCoarse90;
        bidiPhaseAlignmentCoarse91;
        bidiPhaseAlignmentCoarse92;
        bidiPhaseAlignmentCoarse93;
        bidiPhaseAlignmentCoarse94;
        bidiPhaseAlignmentCoarse95;
        bidiPhaseAlignmentCoarse96;
        bidiPhaseAlignmentCoarse97;
        bidiPhaseAlignmentCoarse98;
        bidiPhaseAlignmentCoarse99;
        bidiPhaseAlignmentCoarse100;
        bidiPhaseAlignmentCoarse101;
        bidiPhaseAlignmentCoarse102;
        bidiPhaseAlignmentCoarse103;
        bidiPhaseAlignmentCoarse104;
        bidiPhaseAlignmentCoarse105;
        bidiPhaseAlignmentCoarse106;
        bidiPhaseAlignmentCoarse107;
        bidiPhaseAlignmentCoarse108;
        bidiPhaseAlignmentCoarse109;
        bidiPhaseAlignmentCoarse110;
        bidiPhaseAlignmentCoarse111;
        bidiPhaseAlignmentCoarse112;
        bidiPhaseAlignmentCoarse113;
        bidiPhaseAlignmentCoarse114;
        bidiPhaseAlignmentCoarse115;
        bidiPhaseAlignmentCoarse116;
        bidiPhaseAlignmentCoarse117;
        bidiPhaseAlignmentCoarse118;
        bidiPhaseAlignmentCoarse119;
        bidiPhaseAlignmentCoarse120;
        bidiPhaseAlignmentCoarse121;
        bidiPhaseAlignmentCoarse122;
        bidiPhaseAlignmentCoarse123;
        bidiPhaseAlignmentCoarse124;
        bidiPhaseAlignmentCoarse125;
        bidiPhaseAlignmentCoarse126;
        bidiPhaseAlignmentCoarse127;
        bidiPhaseAlignmentCoarse128;
        bidiPhaseAlignmentCoarse129;
        bidiPhaseAlignmentCoarse130;
        bidiPhaseAlignmentCoarse131;
        bidiPhaseAlignmentCoarse132;
        bidiPhaseAlignmentCoarse133;
        bidiPhaseAlignmentCoarse134;
        bidiPhaseAlignmentCoarse135;
        bidiPhaseAlignmentCoarse136;
        bidiPhaseAlignmentCoarse137;
        bidiPhaseAlignmentCoarse138;
        bidiPhaseAlignmentCoarse139;
        bidiPhaseAlignmentCoarse140;
        bidiPhaseAlignmentCoarse141;
        bidiPhaseAlignmentCoarse142;
        bidiPhaseAlignmentCoarse143;
        bidiPhaseAlignmentCoarse144;
        bidiPhaseAlignmentCoarse145;
        bidiPhaseAlignmentCoarse146;
        bidiPhaseAlignmentCoarse147;
        bidiPhaseAlignmentCoarse148;
        bidiPhaseAlignmentCoarse149;
        bidiPhaseAlignmentCoarse150;
        bidiPhaseAlignmentCoarse151;
        bidiPhaseAlignmentCoarse152;
        bidiPhaseAlignmentCoarse153;
        bidiPhaseAlignmentCoarse154;
        bidiPhaseAlignmentCoarse155;
        bidiPhaseAlignmentCoarse156;
        bidiPhaseAlignmentCoarse157;
        bidiPhaseAlignmentCoarse158;
        bidiPhaseAlignmentCoarse159;
        bidiPhaseAlignmentCoarse160;
        bidiPhaseAlignmentCoarse161;
        bidiPhaseAlignmentCoarse162;
        bidiPhaseAlignmentCoarse163;
        bidiPhaseAlignmentCoarse164;
        bidiPhaseAlignmentCoarse165;
        bidiPhaseAlignmentCoarse166;
        bidiPhaseAlignmentCoarse167;
        bidiPhaseAlignmentCoarse168;
        bidiPhaseAlignmentCoarse169;
        bidiPhaseAlignmentCoarse170;
        bidiPhaseAlignmentCoarse171;
        bidiPhaseAlignmentCoarse172;
        bidiPhaseAlignmentCoarse173;
        bidiPhaseAlignmentCoarse174;
        bidiPhaseAlignmentCoarse175;
        bidiPhaseAlignmentCoarse176;
        bidiPhaseAlignmentCoarse177;
        bidiPhaseAlignmentCoarse178;
        bidiPhaseAlignmentCoarse179;
        bidiPhaseAlignmentCoarse180;
        bidiPhaseAlignmentCoarse181;
        bidiPhaseAlignmentCoarse182;
        bidiPhaseAlignmentCoarse183;
        bidiPhaseAlignmentCoarse184;
        bidiPhaseAlignmentCoarse185;
        bidiPhaseAlignmentCoarse186;
        bidiPhaseAlignmentCoarse187;
        bidiPhaseAlignmentCoarse188;
        bidiPhaseAlignmentCoarse189;
        bidiPhaseAlignmentCoarse190;
        bidiPhaseAlignmentCoarse191;
        bidiPhaseAlignmentCoarse192;
        bidiPhaseAlignmentCoarse193;
        bidiPhaseAlignmentCoarse194;
        bidiPhaseAlignmentCoarse195;
        bidiPhaseAlignmentCoarse196;
        bidiPhaseAlignmentCoarse197;
        bidiPhaseAlignmentCoarse198;
        bidiPhaseAlignmentCoarse199;
        bidiPhaseAlignmentCoarse200;
        bidiPhaseAlignmentCoarse201;
        bidiPhaseAlignmentCoarse202;
        bidiPhaseAlignmentCoarse203;
        bidiPhaseAlignmentCoarse204;
        bidiPhaseAlignmentCoarse205;
        bidiPhaseAlignmentCoarse206;
        bidiPhaseAlignmentCoarse207;
        bidiPhaseAlignmentCoarse208;
        bidiPhaseAlignmentCoarse209;
        bidiPhaseAlignmentCoarse210;
        bidiPhaseAlignmentCoarse211;
        bidiPhaseAlignmentCoarse212;
        bidiPhaseAlignmentCoarse213;
        bidiPhaseAlignmentCoarse214;
        bidiPhaseAlignmentCoarse215;
        bidiPhaseAlignmentCoarse216;
        bidiPhaseAlignmentCoarse217;
        bidiPhaseAlignmentCoarse218;
        bidiPhaseAlignmentCoarse219;
        bidiPhaseAlignmentCoarse220;
        bidiPhaseAlignmentCoarse221;
        bidiPhaseAlignmentCoarse222;
        bidiPhaseAlignmentCoarse223;
        bidiPhaseAlignmentCoarse224;
        bidiPhaseAlignmentCoarse225;
        bidiPhaseAlignmentCoarse226;
        bidiPhaseAlignmentCoarse227;
        bidiPhaseAlignmentCoarse228;
        bidiPhaseAlignmentCoarse229;
        bidiPhaseAlignmentCoarse230;
        bidiPhaseAlignmentCoarse231;
        bidiPhaseAlignmentCoarse232;
        bidiPhaseAlignmentCoarse233;
        bidiPhaseAlignmentCoarse234;
        bidiPhaseAlignmentCoarse235;
        bidiPhaseAlignmentCoarse236;
        bidiPhaseAlignmentCoarse237;
        bidiPhaseAlignmentCoarse238;
        bidiPhaseAlignmentCoarse239;
        bidiPhaseAlignmentCoarse240;
        bidiPhaseAlignmentCoarse241;
        bidiPhaseAlignmentCoarse242;
        bidiPhaseAlignmentCoarse243;
        bidiPhaseAlignmentCoarse244;
        bidiPhaseAlignmentCoarse245;
        bidiPhaseAlignmentCoarse246;
        bidiPhaseAlignmentCoarse247;
        bidiPhaseAlignmentCoarse248;
        bidiPhaseAlignmentCoarse249;
        bidiPhaseAlignmentCoarse250;
        bidiPhaseAlignmentCoarse251;
        
    end
    
    %Hidden PDEP properties created by this class
    properties (GetObservable,SetObservable,Hidden)
        %         droppedFramesTotal;  % async thread MEX dropped frames (single frame buffer)
        %         droppedLogFramesTotal;  % loggng thread MEX dropped frames
        droppedFramesLast;
        droppedLogFramesLast;
        droppedProcessedFramesLast;
        
        frameTagEnable; %Logical. If true, frame tagging is used to identify each frame copied from the LSM.
        
        %Property must be hidden to avoid nested header string
        loggingHeaderString=''; %String containing header information to store as metadata in logging TIF file
    end
    
    properties (Hidden)
        verbose = false;
        loggingOpenModeString='wbn';   % the mode string passed to fopen when opening the log file
    end
    
    properties (SetAccess=protected, Hidden, Dependent)
        numChannelsActive; %Number of channels currently active
        loggingFullFileName;  % the complete path and file name of the file to write to, including extension
    end
    
    %Flag properties
    properties (SetAccess=protected,Hidden)
        paramChangeList={}; %Cell array of properties changed since last call to Thor API StartAcquisition() function
        loggingFileRolloverFlag=false;
        initialized = false;
        allowLogging=false; %Can only be set during start() method calls.
    end
    
    properties (Constant, Hidden)
        %framePeriodOverhead = 6.06e-3; %Time, in ms, that is added to each frame period as overhead -- mostly due to the Y scanner flyback time
        
        loggingFrameTagEnable = true;
        loggingFrameTagOneBased = true; %If true, frame tags are converted to 1-based indexing
                
        CONFIG_BUFFER_PARAMS = {'pixelsPerLine' 'linesPerFrame' 'channelsLogging' 'channelsViewing' 'circBufferSize' 'loggingAveragingFactor' 'dataMappingMode' 'aspectRatioY' 'areaMode' 'subtractChannelOffsets'}; 
    end
    
    
    %% CONSTRUCTOR/DESTRUCTOR
    methods
        
        function obj = LSM(varargin)
            
            %Invoke superclass constructor
            obj = obj@dabs.thorlabs.private.ThorDevice(varargin{:});
            
            %Construct/identify (required) associated PMT object
            obj.hPMTModule = dabs.thorlabs.PMT();
            
            %Determine number of channels & initialize arrays of length equal to this number
            channelsActiveInfo = obj.paramInfoMap('channelsActive');
            obj.numChannelsAvailable = log(channelsActiveInfo.paramMax + 1)/log(2);
            obj.channelsLogging = [true; false(obj.numChannelsAvailable-1,1)];
            obj.channelsViewing = [true; false(obj.numChannelsAvailable-1,1)];
            obj.subtractChannelOffsets = false(obj.numChannelsAvailable,1);
            obj.channelOffsets = zeros(obj.numChannelsAvailable,1);
            
            %Activate frame-tagging, if available (Thor API 1.3 and later)
            if ~isempty(obj.frameTagEnable)
                obj.frameTagEnable = true;
            end
            
            %Invoke superclass initializer
            obj.initialize();
            
            %             %Handle initialization of inputChannelRange properties in special way
            %             %Addresses weird behavior/bug where channel offset depends somehow on history of channel ranges
            %             enumValMapMap = obj.accessAPIDataVar('enumValMapMap');
            %             enumValMap = enumValMapMap('InputRange');
            %             for i=1:obj.numChannelsAvailable
            %                 paramName = sprintf('inputChannelRange%d',i);
            %
            %                 maxVal = enumValMap(obj.paramInfoMap(paramName).paramMax);
            %                 defaultVal = enumValMap(obj.paramInfoMap(paramName).paramMax);
            %
            %                 %Switches inputChannelRange property away from, and back to, its default value
            %                 obj.(paramName) = maxVal;
            %                 obj.(paramName) = defaultVal;
            %             end
            %
            %Initialize MEX interface
            obj.configureFrameAcquiredEvent('initialize');
            obj.configureFrameAcquiredEvent('configBuffers');
            
            %Initialize flags
            obj.initialized = true;
            obj.paramChangeList = {''}; %Set to dummy value, so not empty
        end
        
        function delete(obj)
            if(strcmpi(obj.state,'active'))
                obj.stop();
            end
            
            %obj.apiCall('TeardownCamera');
            obj.configureFrameAcquiredEvent('destroy');
            
            delete(obj.hPMTModule);
            
            %                unloadlibrary('ThorConfocal');
        end
    end
    
    
    %% PROPERTY ACCESSS
    methods
        
        function val = get.bidiPhaseAlignmentCoarse(obj)
            zone = obj.fieldSizeMax - obj.fieldSize + 1;
            val = obj.(sprintf('bidiPhaseAlignmentCoarse%d',zone));
        end
        
        function set.bidiPhaseAlignmentCoarse(obj,val)
            zone = obj.fieldSizeMax - obj.fieldSize + 1;
            obj.(sprintf('bidiPhaseAlignmentCoarse%d',zone)) = val;
        end
        
        function val = get.numChannelsActive(obj)
            val =  length(find(obj.channelsActive));
        end
        
        function fName = get.loggingFullFileName(obj)
            [p,f,e] = fileparts(obj.loggingFileName);
            
            if isempty(p)
                p = pwd();
            end
            
            if isempty(e)
                e = '.tif';
            end
            
            fName = fullfile(p,[f e]);
        end
        
        
        function val = get.loggingFrameDelayMax(obj)
            val = round(obj.circBufferSize/2);
        end
        
        function val = get.signedData(obj)
            switch obj.dataMappingMode
                case 'POLARITY_INDEPENDENT'
                    val = false;
                case {'POLARITY_POSITIVE' 'POLARITY_NEGATIVE'}
                    val = true;
            end
        end
        
        %NOTE: Following are now PDEP properties, but these can be restored to test/compare execution time as pure dependent property, vs pdep property.
        %      Tests found get-access time of fieldSize as 5.9ms with PDEP and 4.8ms with pure dependent (12/14/2010)
        %
        %         function val = get.fieldSize(obj)
        %             val = obj.apiCall('GetParam',obj.paramCodeMap('fieldSize'),0);
        %
        %         end
        %
        %         function set.fieldSize(obj,val)
        %
        %             obj.apiCall('SetParam',obj.paramCodeMap('fieldSize'),val);
        %
        %             if(obj.running && obj.callMEXOnParamChange)
        %                 obj.configureFrameAcquiredEvent('setup');
        %                 obj.configureFrameAcquiredEvent('start');
        %             end
        %         end
        %
        
        
        %         function set.channelsActive(obj,val)
        %             %            if(obj.running)
        %             %                error('LSM: cannot change channels while device is running');
        %             %            end
        %            obj.pdepSetAssert(val,isnumeric(val) && (isvector(val) || isempty(val)) && all(ismember(val,1:obj.numChannelsAvailable)),'Invalid value entered for ''channelsActive''');
        %            obj.channelsActive = val;
        %            end
        
        %         function set.circBufferSize(obj,val)
        %             obj.circBufferSize = val;
        %             if(obj.callMEXOnParamChange)
        %                 obj.configureFrameAcquiredEvent('configBuffers');
        %             end
        %         end
        
        function set.subtractChannelOffsets(obj,val)
            validateattributes(val,{'logical'},{'vector' 'numel' obj.numChannelsAvailable});
            obj.subtractChannelOffsets = val;
        end
        
        
        function set.loggingAutoStart(obj,val)
            assert(isscalar(val) && (islogical(val) || ismember(val,[0 1])),'Property ''loggingAutoStart'' must be a logical scalar');
            obj.loggingAutoStart = val;
        end
        
        function set.allowLogging(obj,val)
            val = logical(val);
            assert(isscalar(val),'Property ''allowLogging'' must be a scalar logical value');
            obj.allowLogging = val && any(obj.channelsLogging);
        end
        
        %         function set.loggingEnable(obj,val)
        %             obj.loggingEnable = logical(val);
        %             if(obj.running)
        %                 obj.loggingEnableMEX = obj.loggingEnable;
        %             end
        %         end
        %
        %         function set.loggingEnableMEX(obj,val)
        %
        %             obj.loggingEnableMEX = val;
        %             dbstack
        %             obj.configureFrameAcquiredEvent('configLogFile');
        %         end
        
        %         function set.loggingHeaderString(obj,val)
        %             assert(most.idioms.isstring(val),'The value of loggingHeaderString must be a string');
        %             assert(~obj.running || obj.loggingStreamUpdateFlag, 'Value of loggingHeaderString must be set via updateLoggingStream() while logging is active');
        %
        %             obj.loggingHeaderString =    val;
        %         end
        
        
        function set.frameAcquiredEventFcn(obj,val)
            obj.frameAcquiredEventFcn = val;
            if(obj.initialized)
                obj.configureFrameAcquiredEvent('configCallback');
            end
        end
        
    end
    
    %PDep Property Handling
    methods (Hidden, Access=protected)
        function pdepPropHandleGet(obj,src,evnt)
            propName = src.Name;
            
            switch propName
                
                %Properties with string-encoded values
                case {'triggerMode' 'scanMode' 'averagingMode' 'areaMode' 'dataMappingMode'}
                    obj.pdepPropGroupedGet(@obj.getParameterEncoded,src,evnt);
                    
                case {'inputChannelRange1' 'inputChannelRange2' 'inputChannelRange3' 'inputChannelRange4'}
                    obj.pdepPropGroupedGet(@obj.getInputChannelRange,src,evnt);
                    
                    %Properties with a maximum-value encoded as Inf by this class
                case {'triggerTimeout'}
                    obj.pdepPropGroupedGet(@obj.getParameterMaxInf,src,evnt);
                    
                    %Properties that are maintained in MEX function
                case {'framesAvailable' 'frameCount' 'droppedFramesLast' 'droppedLogFramesLast' 'droppedProcessedFramesLast'}
                    obj.pdepPropGroupedGet(@obj.getMEXProperty, src, evnt);
                    
                    %Properties to get directly. Do nothing -- simple pass-through
                case {'circBufferSize' 'loggingFileName' 'loggingHeaderString' 'loggingAveragingFactor' 'frameEventDecimationFactor' 'channelsLogging' 'channelsViewing' 'subtractChannelOffsets' 'channelOffsets'}
                    
                case {'channelsActive' 'multiFrameCount'}
                    obj.pdepPropIndividualGet(src, evnt);
                    
                otherwise
                    obj.pdepPropGroupedGet(@obj.getParameterSimple,src,evnt);
            end
        end
        
        function pdepPropHandleSet(obj,src,evnt)
            propName = src.Name;
            
            disallowWhileRunning = {'triggerMode' 'multiFrameCount' 'channelsLogging' 'channelsViewing' 'circBufferSize' 'loggingAveragingFactor' 'frameEventDecimationFactor' 'dataMappingMode' 'captureWithoutScanner' 'areaMode'};
            needsConfigBuffers = {'pixelsPerLine' 'linesPerFrame' 'channelsLogging' 'channelsViewing' 'circBufferSize' 'loggingAveragingFactor' 'dataMappingMode' 'aspectRatioY' 'areaMode' 'subtractChannelOffsets'}; %These properties require that MEX-maintained buffers be reconfigured
            alwaysAllow  = {'loggingFileName' 'loggingHeaderString'};
            
            if strcmpi(obj.state,'active') && ~ismember(propName, alwaysAllow)
                if ismember(propName, disallowWhileRunning)
                    error('Cannot set ''%s'' while acquisition is running', propName);
                elseif ismember(propName, needsConfigBuffers) && obj.logging
                    error('Cannot set ''%s'' while acquisition is running and logging to disk', propName);
                elseif ~strcmpi(obj.pdepGetDirect('triggerMode'),'SW_FREE_RUN_MODE')
                    error('Cannot set ''%s'' when running in single-frame or multi-frame triggerMode', propName);
                end
            end
            
            switch propName
                
                case {'pixelsPerLine' 'linesPerFrame' 'areaMode'}
                    obj.pdepPropGroupedSet(@obj.setPixelationParameter,src,evnt)
                    
                case {'triggerMode' 'scanMode' 'averagingMode' 'inputChannelRange1' 'inputChannelRange2' 'inputChannelRange3' 'inputChannelRange4' 'dataMappingMode'}
                    obj.pdepPropGroupedSet(@obj.setParameterEncoded,src,evnt);
                    
                case {'multiFrameCount' 'frameEventDecimationFactor'}
                    obj.pdepPropIndividualSet(src,evnt);
                    
                case {'triggerTimeout'}
                    obj.pdepPropGroupedSet(@obj.setParameterMaxInf,src,evnt);
                    
                case {'channelsActive' 'frameCount' 'framesAvailable' 'droppedFramesLast' 'droppedLogFramesLast'}
                    %Read-only properties
                    obj.pdepPropSetDisallow(src,evnt);
                    return;
                    
                case {'circBufferSize' 'loggingAveragingFactor' 'subtractChannelOffsets' 'channelOffsets'}
                    %Do nothing -- pass-through
                    
                case {'loggingHeaderString' 'loggingFileName'}
                    %Properties related to logging configuration -- NOT Thor API 'parameters'
                    obj.pdepPropGroupedSet(@obj.setLoggingProperty,src,evnt);
                    return;
                    
                case {'channelsLogging' 'channelsViewing'}
                    %Properties specifying number of channels to be loggable/viewable -- not direct Thor API 'parameters', but do set channelsActive parameter indirectly
                    obj.pdepPropGroupedSet(@obj.setNumChansProperty,src,evnt);
                    
                otherwise
                    obj.pdepPropGroupedSet(@obj.setParameterSimple,src,evnt);
            end
            
            %No further action for 'touchParameter'
            if strcmpi(propName,'touchParameter')
                return;
            end
            
            %Signal that a parameter has changed
            obj.paramChangeList{end+1} = propName;            
            
            %Restart already-running acquisition, if needed
            if obj.initialized
                if strcmpi(obj.state,'active') && obj.restartOnParamChange
                    obj.startAlreadyRunning();
                elseif ismember(propName,obj.CONFIG_BUFFER_PARAMS)
                    obj.configureFrameAcquiredEvent('configBuffers');
                end
            end
            
        end
    end
    
    methods (Hidden)
        
        function val = getMEXProperty(obj,propName)
            val = obj.configureFrameAcquiredEvent('get',propName);
        end
        
        function val = getMultiFrameCount(obj)
            val = obj.getParameterSimple('multiFrameCount');
            if val >= intmax('int32')
                val = inf;
            end
        end
        
        function val = getInputChannelRange(obj,propName)
            rawVal = obj.getParameterSimple(propName);
            
            %Convert raw (numeric) value to corresponding string
            enumValMapMap = obj.accessAPIDataVar('enumValMapMap');
            enumValMap = enumValMapMap('InputRange');
            
            val = enumValMap(rawVal);  %Converts to string corresponding to value
            
        end
        
        function val = getChannelsActive(obj)
            %Unpack the scalar into a vector
            scalarVal = obj.apiCall('GetParam', obj.paramCodeMap('channelsActive'),0);
            val = find(fliplr(dec2bin(scalarVal,obj.numChannelsAvailable))==49);
        end
        
        %         function setChannelsActive(obj,val)
        %             %Pack vector value into a scalar
        %             obj.apiCall('SetParam', obj.paramCodeMap('channelsActive'), sum(2.^(val-1)));
        %
        %             %Dependencies
        %             obj.multiFrameCount = obj.multiFrameCount;
        %         end
        
        function setFrameEventDecimationFactor(obj,val)
            if obj.initialized
                obj.configureFrameAcquiredEvent('configCallbackDecimationFactor');
            end
        end
        
        function setLoggingProperty(obj,propName,val)
            assert(most.idioms.isstring(val),'The value of ''%s'' must be a string',propName);
            assert(~obj.logging || obj.loggingFileRolloverFlag,'Value of ''%s'' can only be set in idle or armed states, or via the rolloverLogFile() method when active');
            
            %
            %             %assert(~obj.running || obj.loggingStreamUpdateFlag, 'Value of ''%s'' must be set via updateLoggingStream() while logging is active',propName);
            %
            %              %Allow changes to occur directly while running, except if called from updateLoggingStream()
            %              % When not running or when called from updateLoggingStream() --> defer call to MEX file
            %              if obj.running && ~obj.loggingStreamUpdateFlag
            %                  dbstack;
            %                  obj.configureFrameAcquiredEvent('configLogFile');
            %              end
        end
        
        function setMultiFrameCount(obj,val)
            val = min(val,intmax('int32'));
            obj.setParameterSimple('multiFrameCount',val);
        end
        
        function setNumChansProperty(obj,propName,val)
            val = logical(val); %throws if not convertible to logical
            assert(numel(val) == obj.numChannelsAvailable && isvector(val),'Value of %s must be a vector of length %d',propName,obj.numChannelsAvailable);
            
            channelsLoggingVal = obj.channelsLogging;
            channelsViewingVal = obj.channelsViewing;
            
            %Handle initialization case
            if isempty(channelsLoggingVal) || isempty(channelsViewingVal)
                return;
            end
            
            %Determine active channels
            channelsActiveBitMask = channelsLoggingVal | channelsViewingVal;
            channelsActiveVector = find(channelsActiveBitMask);
            
            %Pack vector value into a scalar; set 'channelsActive' param/property
            obj.apiCall('SetParam', obj.paramCodeMap('channelsActive'), sum(2.^(channelsActiveVector-1)));
            
            %Dependencies
            obj.multiFrameCount = obj.multiFrameCount;
        end
        
        function setPixelationParameter(obj,propName,val)
            ppl = obj.pdepGetDirect('pixelsPerLine');
            lpf = obj.pdepGetDirect('linesPerFrame');
            am = obj.pdepGetDirect('areaMode');
            
            switch propName
                case 'pixelsPerLine'
                    ppl = val;
                case 'linesPerFrame'
                    lpf = val;
                case 'areaMode'
                    am = val;
                otherwise
                    assert(false);
            end
            
            obj.setParameterEncoded('areaMode',am);
            
            %             if setLPFFirst
            %                 obj.setParameterSimple('linesPerFrame',lpf);
            %                 pause(1);
            %                 obj.setParameterSimple('pixelsPerLine',ppl);
            %             else
            obj.setParameterSimple('pixelsPerLine',ppl);
            obj.setParameterSimple('linesPerFrame',lpf);
            
            %pause(1);
            %end
        end
        
        %         function setLoggingProperty(obj,propName,~) %(obj,propName,val)
        %             %Re-configure the log file
        %             if obj.callMEXOnParamChange
        %                 assert(~obj.running || obj.loggingStreamUpdateFlag, 'Value of property ''%s'' must be set via updateLoggingStream() while logging is active', propName);
        %
        %                 if ~obj.loggingStreamUpdateFlag %If calling from updateLoggingStream(), defer call to MEX file
        %                     obj.configureFrameAcquiredEvent('configLogFile');
        %                 end
        %             end
        %         end
        
    end
    
    
    %% PUBLIC METHODS
    
    methods
        
        function rearm(obj)
            %Rearm previously armed/started acquisition.
            %Used to allow hardware-triggered acquisition to be retriggered for a new set of acquired frames.
            
            assert(strcmpi(obj.state,'active'),'Method rearm() must be called during active acquisition');
            obj.configureFrameAcquiredEvent('setup');
        end
        
        
        function arm(obj)
            %Macro method used to arm an acquisition.
            %Calls both PreflightAcquisition() & SetupAcquisition() Thor API functions, prepares log file, resets acquisition flags, etc.
            
            if strcmpi(obj.state,'active')
                obj.stop();
            end
            
            if ~isempty(obj.paramChangeList)
                err = obj.configureFrameAcquiredEvent('preflight');
                if(~err)
                    msg = uint16(ones(1, 64));
                    [status, msg] = obj.apiCallRaw('GetLastErrorMsg', msg, 64);
                    
                    error('Error occurred during call to PreflightAcquisition: %s', msg);
                end
                obj.paramChangeList = {};
            end
            
            obj.framesGotten = 0;
            
            %obj.loggingEnableMEX = obj.loggingEnable;  % this calls to configLogFile -- property values of loggingHeaderString and loggingFileName are enforced
            
            obj.configureFrameAcquiredEvent('newacq'); %Stops ThorFrameCopier thread and arms it for new acquisition (ThorFrameCopier stopProcessing() & arm())
            obj.configureFrameAcquiredEvent('setup');
            
            obj.state = 'armed';
        end
        
        %         function armLogging(obj)
        %             %Method to arm logging, which may be done before or after arming/starting an acquisition
        %             %Arming logging /after/ start of acquisition can allow headerString to be fully
        %
        %         end
        
        
        function [data,frameTags] = getData(obj,numFrames)
            %Retrieves available frame image data and frame tag data from LSM, up to specified numFrames
            % numFrames: <Default=inf> Maximum number of available frames to retrieve data from
            % data: Image data, returned as MxNxCxK array representing K frames of MxN pixels and C channels each
            % frameTags: If frameTagEnable=true, an array of Kx1 frame tag values indicating the ordinal frame number stored with each of the retrieved frames.
            %                        If frameTagEnable=false, an empty array is returned
            
            %            disp('LSM.getdata: calling configureFrameAcquiredEvent');
            
            if nargin < 2 || isinf(numFrames)
                data = obj.configureFrameAcquiredEvent('getdata');
            else
                data = obj.configureFrameAcquiredEvent('getdata',numFrames);
            end
            
            frameTags = [];
            
            if ~isempty(data)
                if iscell(data) %implies frameTagEnable=true
                    frameTags = data{2} + 1; %Convert to 1-based indexing
                    data = data{1};
                end
                
                sz = size(data);
                
                if numel(sz) > 3
                    obj.framesGotten = obj.framesGotten + sz;
                else
                    obj.framesGotten = obj.framesGotten + 1;
                end
            end
        end
        
        % This is a semi-temporary fix for SI4.frameAcquiredFcn. This is
        % bad code, it dupes logic from CFAE.
        %
        % if tfSuccess==true, data is an actual frame from CFAE.
        % if tfSuccess=false, data is a dummy placeholder frame.
        function [tfSuccess data frameTags] = getDataWithDummyFrame(obj,numFrames)
            if nargin==1
                [data, frameTags] = obj.getData();
            else
                [data, frameTags] = obj.getData(numFrames);
            end
            
            if ~isempty(data)
                tfSuccess = true;
            else
                tfSuccess = false;
                
                % Failed to get frame(s). Create dummy frame as
                % placeholder. Always use a single dummy frame even if
                % numFrames>1.
                
                % cut+paste from CFAE
                if obj.numChannelsActive==1
                    frameNumChans = 1;
                else
                    frameNumChans = obj.numChannelsAvailable;
                end
                
                if obj.signedData
                    dataType = 'int16';
                else
                    dataType = 'uint16';
                end
                
                data = zeros(obj.linesPerFrame,obj.pixelsPerLine,frameNumChans,dataType); % class is hardcoded in CFAE
                for c = 1:size(data,3)
                    % init each channel image to half black/white
                    data(:,1:round(end/2),c) = intmax(dataType);
                end
                
                frameTags = nan; %signify this is a dummy frame
            end
        end
        
        function start(obj,allowLogging)
            %Starts scanner and armed acquisition
            
            assert(strcmpi(obj.state,'armed'),'Acquisition must be armed before it can be started');
            
            if nargin > 1
                obj.allowLogging = allowLogging;
            end
            
            if obj.loggingAutoStart && any(obj.channelsLogging) && obj.allowLogging
                obj.startLogging();
            end
            
            if ~obj.captureWithoutScanner
                obj.hPMTModule.scanEnable = 1; %Actually starts the scanner
            end
            
            obj.configureFrameAcquiredEvent('start',obj.allowLogging); %Starts acquisition thread and LSM acquisition
            
            obj.state = 'active';
        end
        
        
        function startLogging(obj,frameDelay)
            %Starts file logging, for either an armed or ongoing acquisition
            %   frameDelay: Number of frames by which to delay logging. Note that value is capped by (circBufSize/2).
            
            assert(~obj.logging,'Logging has already been started');
            assert(ismember(obj.state,{'armed' 'active'}),'Method can only be called when in ''armed'' or ''active'' state');
            
            if nargin < 2
                frameDelay = 0;
            end
            validateattributes(frameDelay,{'numeric'},{'scalar' 'nonnegative' 'integer' 'finite'},'','frameDelay');
            
            maxFrameDelay = obj.loggingFrameDelayMax;
            if frameDelay > maxFrameDelay
                fprintf(2,'WARNING (%s): Frame delay specified (%d) exceeded maximum allowed value (%d) and has been capped at such\n',mfilename('class'),frameDelay,maxFrameDelay);
                frameDelay = maxFrameDelay;
            end
            
            obj.configureFrameAcquiredEvent('configLogFile');
            obj.configureFrameAcquiredEvent('startLogger',frameDelay);
            
            obj.logging = true;
        end
        
        function rolloverLogFile(obj,frameToRollover,varargin)
            %Begin logging to new file at specified frameToRollover. The new file is determined by the properties {'loggingFullFileName' 'loggingOpenModeString' 'loggingHeaderString'}, which can optionally be set by this method via property-value pairs.
            
            assert(obj.logging,'Method can only be called if file logging is already ongoing');
            
            if ~isempty(varargin)
                p = varargin(1:2:end);
                v = varargin(2:2:end);
                assert(iscellstr(p) && length(p) == length(v),'Invalid property-value pair specification');
                assert(all(ismember(p,{'loggingFileName' 'loggingOpenModeString' 'loggingHeaderString'})), 'Property specified in prop-value pairs cannot be updated by this method');
                
                obj.loggingFileRolloverFlag = true;
                try
                    for i=1:length(p)
                        obj.(p{i}) = v{i};
                    end
                catch ME
                    obj.loggingFileRolloverFlag = false;
                    ME.rethrow();
                end
                
                obj.loggingFileRolloverFlag = false;
            end
            
            obj.configureFrameAcquiredEvent('addLogfileRolloverNote',frameToRollover);
        end
        
        function processedFrameQDrops = stop(obj,suppressWarnings)
            %Stop scanning/acquisition/logging immediately -- any queued frames not logged are lost.
            %  suppressWarnings: <Default=false> If true, warning messages that appear when loggingQDrops and/or thorFrameDrops are detected are suppressed
            
            if nargin < 2
                suppressWarnings = false;
            end
            
            processedFrameQDrops = obj.stopOrFinish('stop',suppressWarnings);
        end
        
        function processedFrameQDrops = finish(obj,suppressWarnings)
            %Stop scanning/acquisition immediately. Waits for any queued frames to be logged and then stops logging.
            %  suppressWarnings: <Default=false> If true, warning messages that appear when loggingQDrops and/or thorFrameDrops are detected are suppressed
            
            if nargin < 2
                suppressWarnings = false;
            end
            
            processedFrameQDrops = obj.stopOrFinish('finish',suppressWarnings);
        end
        
        function startAlreadyRunning(obj,suppressWarning)
            %Restart currently active acquisition, as is required for an LSM parameter change to take effect
            %Restart action will reset counts - frame tag, # dropped frames, etc - and call Thor API functions SetupAcquisition() & StartAcquistion()
            %
            % suppressWarning: <Default=false> If true, warning when called in idle state is suppressed
            if strcmpi(obj.state,'active')
                
                if (~isempty(obj.paramChangeList) && any(ismember(obj.paramChangeList,obj.CONFIG_BUFFER_PARAMS)))
                    %Handle parameter changes which require call to configBuffers. This requires
                    obj.configureFrameAcquiredEvent('stop'); %This stops ThorFrameCopier processing; does NOT call postflightAcquisition()
                    obj.configureFrameAcquiredEvent('configBuffers');
                    obj.configureFrameAcquiredEvent('setup');
                    obj.configureFrameAcquiredEvent('start',obj.allowLogging);
                else 
                    if isempty(obj.paramChangeList) %Force setup/startAcquisition calls to have full restart effect even if no parameter has been changed
                       obj.touchParameter = 1; 
                    else 
                        celldisp(obj.paramChangeList);
                    end
                    obj.configureFrameAcquiredEvent('startAlreadyRunning'); %Calls SetupAcquisition() & StartAcquisition(); resets counters - frame tag, dropped frames, etc
                end
            elseif nargin < 2 || ~suppressWarning
                fprintf(2,'WARNING (%s): Scan is not active. Method startAlreadyRunning() has no effect.\n');
            end
        end
        
        function parkAtCenter(obj)
            
            assert(ismember(obj.state,{'idle' 'armed'}),'Cannot park scanner while it is already active.');
            
            obj.scanMode = 'SCAN_MODE_CENTER';
            obj.triggerMode = 'SW_FREE_RUN_MODE';
            obj.arm();
            
            %Start LSM, without starting scanner (PMT property)
            obj.configureFrameAcquiredEvent('start',false); %Starts acquisition thread and LSM acquisition
            
            obj.state = 'active';
        end
        
        function pause(obj,stopScan)
            %Stop scanning/acquisition, but allow it to be subsequently resumed
            %Resumed acquisitions continue logging data to same file
            %  stopScan: <Default=false> If true, stop scanning until resumed. Otherwise scanning is continued, although acquisition is stopped.
            
            obj.configureFrameAcquiredEvent('pause');
            
            if nargin > 1 && stopScan
                obj.hPMTModule.scanEnable = 0;
            end
        end
        
        function resume(obj)
            %Resumes scanning/acquisition that was previously paused
            %Resumed acquisitions continue logging data to same file
            
            %obj.configureFrameAcquiredEvent('finishLogging');
            %obj.start();
            if obj.hPMTModule.scanEnable == 0
                obj.hPMTModule.scanEnable = 1;
            end
            obj.configureFrameAcquiredEvent('resume');
        end
        
        function readOffsets(obj)
            channelsViewingCache = obj.channelsViewing;
            frameAcquiredFcnCache = obj.frameAcquiredEventFcn;
            
            ME = [];
            try
                obj.channelsViewing = true(obj.numChannelsAvailable,1);
                obj.triggerMode = 'SW_SINGLE_FRAME';
                obj.captureWithoutScanner = true;
                obj.arm();
                obj.start(false);
                
                t = tic;
                timeoutTime = 2;
                timeout = false;
                
                while obj.frameCount < 1
                    pause(0);
                    
                    if toc(t) > timeoutTime
                        timeout = true;
                        break;
                    end
                end
                
                obj.stop();
                
                if timeout
                    fprintf(2,'WARNING (%s): Failed to read channel offsets within %d s. Channel offset values not updated.\n',class(obj),timeoutTime);
                    
                else
                    frameData = obj.getData(1); %Read one frame
                    
                    %Compute new offset, accounting for any previous offset subtraction
                    for i=1:obj.numChannelsAvailable
                        obj.channelOffsets(i) = round(mean(mean(frameData(:,:,i)))) + obj.channelOffsets(i) * obj.subtractChannelOffsets(i);
                    end
                    
                    obj.configureFrameAcquiredEvent('configBuffers');
                end
                
            catch MEtemp
                ME = MEtemp;
            end
            
            obj.captureWithoutScanner = false;
            obj.frameAcquiredEventFcn = frameAcquiredFcnCache;
            obj.channelsViewing = channelsViewingCache;
            
            if ~isempty(ME)
                ME.rethrow();
            end
        end
        
        function tf = isAcquiring(obj)
            tf = obj.configureFrameAcquiredEvent('isAcquiring');
        end
        
        function flushData(obj)
            obj.configureFrameAcquiredEvent('flush');
        end
        
        %         function updateLoggingStream(obj,loggingFileName,loggingHeaderString,nextFileFrameCount)
        %             %Method allowing loggingFileName and, optionally, loggingHeaderString, to be updated during a running acquisition
        %
        %             try
        %                 obj.loggingStreamUpdateFlag = true;
        %
        %                 obj.loggingFileName = loggingFileName;
        %                 if nargin >= 3
        %                     obj.loggingHeaderString = loggingHeaderString;
        %                 end
        %
        %                 dbstack
        %                 if nargin >= 4
        %                     obj.configureFrameAcquiredEvent('configLogFile',nextFileFrameCount);
        %                 else
        %                     obj.configureFrameAcquiredEvent('configLogFile');
        %                 end
        %
        %                 obj.loggingStreamUpdateFlag = false;
        %
        %             catch ME
        %                 obj.loggingStreamUpdateFlag = false;
        %                 ME.rethrow();
        %             end
        %         end
        %
        %
        %         function abortAcquisition(obj)
        %             %Macro method used to abort an ongoing acquisition
        %
        %
        %         end
        
        %         % used for single frame mode, this starts an acquisition without
        %         % resetting params, etc.  start() must have been called prior to
        %         % calling this function, or else it does not
        %         function nextFrame(obj)
        %             if(~obj.running)
        %                 error('LSM: call start() before calling nextFrame()');
        %             end
        %
        %             obj.configureFrameAcquiredEvent('start');
        %         end
        
        %         function start(obj)
        %             if(obj.running)
        %                 obj.stop();
        %             end
        %
        %             % force sync of input channel ranges
        %             obj.apiCall('SetParam',obj.paramCodeMap('inputChannelRange1'),obj.inputChannelRange1);
        %             obj.apiCall('SetParam',obj.paramCodeMap('inputChannelRange2'), obj.inputChannelRange2);
        %
        %             if(obj.paramChangeFlag)
        %                 err = obj.configureFrameAcquiredEvent('preflight');
        %                 disp(['LSM.start() preflight returned ' num2str(err)]);
        %                 if(~err)
        %                     msg = char(ones(1, 64));
        %                     [status, msg] = obj.apiCallRaw('GetLastErrorMsg', msg, 64);
        %                     disp(['Error msg=' msg]);
        %                     return
        %                 end
        %                 obj.paramChangeFlag = false;
        %             end
        %
        %             obj.framesGotten = 0;
        %             obj.droppedFrames = 0;
        %             obj.hPMTModule.scanEnable = 1;
        %             obj.loggingEnableMEX = obj.loggingEnable;  % ensure call to configLogFile
        %
        %             obj.running = true;
        %
        %             obj.configureFrameAcquiredEvent('newacq');
        %
        %             obj.configureFrameAcquiredEvent('setup');
        %             obj.configureFrameAcquiredEvent('start');
        %         end
        %
        %
        
    end
    
    %% PRIVATE/PROTECTED METHODS
    
    
    methods (Hidden)
        
        function preflightAcquisition(obj)
            %Direct method to arm acquisition with current settings and resets DAQ board
            %obj.configureFrameAcquiredEvent('configBuffers');
            obj.configureFrameAcquiredEvent('preflight');
        end
        
        function setupAcquisition(obj)
            %Direct method to arm acquisition with current settings without resetting DAQ board
            %Unlike preflightAcquisition(), setupAcquisition() can be called in midst of ongoing acquisition
            
            obj.configureFrameAcquiredEvent('setup');
        end
        
        %         function startAcquisition(obj)
        %             %Starts pre-configured acquisition
        %
        %             obj.configureFrameAcquiredEvent('startDirect');
        %         end
        
        function postflightAcquisition(obj)
            %Stops ongoing acquisition, releasing resources
            obj.configureFrameAcquiredEvent('postflight');
        end
        
        function status = statusAcquisition(obj)
            %Returns the status of the acquisition
            
            status = obj.apiCall('StatusAcquisition', 0);
            %TODO: Decode status
        end
        
        function [status, lastCompletedFrameIndex] = statusAcquisitionEx(obj)
            %Returns status of acquisition and frame count maintained by scanner driver
            %   lastCompletedFrameIndex: Index of the last known frame to be available for collection
            
            [status, lastCompletedFrameIndex] = obj.apiCall('StatusAcquisitionEx', 0, 0);
            %TODO: Decode status
        end
        
    end
    
    methods (Access=protected)
        
        function [processedQDrops, loggingQDrops, thorFrameDrops] = stopOrFinish(obj,cmdString,suppressWarnings)
            %   cmdString: One of {'stop' 'finish'}
            %   suppressWarnings: <Default=false> If true, warning messages that appear when loggingQDrops and/or thorFrameDrops are detected are suppressed
            
            assert(ismember(cmdString,{'stop' 'finish'}));
            %assert(strcmpi(obj.state,'active'),'The ''%s'' object is not ''active'' -- ignoring stop/finish command',mfilename('class'));
            
            %VI031612: Do we need to assert that we're active, or can these be called safely regardless of current state?
            obj.hPMTModule.scanEnable = 0;
            obj.configureFrameAcquiredEvent(cmdString);
            
            processedQDrops = 0;
            loggingQDrops = 0;
            thorFrameDrops = 0;
            
            if strcmpi(obj.state,'active') %Calling postflight() when you didn't just complete an acquisition causes issues with subsequent acquisitions (no frame clock appears)
                obj.configureFrameAcquiredEvent('postflight');
                
                %Display dropped frame warnings, as needed
                if ~obj.verbose %At moment, MEX layer provides diagnostic info in 'verbose' mode
                    thorFrameDrops = obj.droppedFramesLast;
                    
                    loggingQDrops = 0;
                    if obj.logging
                        loggingQDrops = obj.droppedLogFramesLast;
                    end
                    
                    if thorFrameDrops
                        fprintf(2,'WARNING: Frames Dropped! Failed to copy %d frames from ''%s'' driver during last acquisition.\n', thorFrameDrops, obj.apiPrettyName);
                    end
                    
                    if loggingQDrops
                        fprintf(2,'WARNING: Dropped logging frames! Dropped %d frames in disk logging stream during last acquisition (file is missing these frames).\n', loggingQDrops);
                    end
                    
                end
                
                %Return dropped processed frame count, so client classes can warn if desired
                processedQDrops = obj.droppedProcessedFramesLast;
                
            end
            
            obj.state = 'idle';
            obj.logging = false;
        end
    end
    
    
    
end

%% HELPERS


function prop2ParamMap = zlclInitProp2ParamMap()

prop2ParamMap = containers.Map('KeyType','char','ValueType','char');

prop2ParamMap('triggerMode') = 'PARAM_TRIGGER_MODE';
prop2ParamMap('multiFrameCount') = 'PARAM_MULTI_FRAME_COUNT';
prop2ParamMap('cameraType') = 'PARAM_CAMERA_TYPE';
prop2ParamMap('pixelsPerLine') = 'PARAM_LSM_PIXEL_X';
prop2ParamMap('linesPerFrame') = 'PARAM_LSM_PIXEL_Y';
prop2ParamMap('fieldSize') = 'PARAM_LSM_FIELD_SIZE';
prop2ParamMap('channelsActive') = 'PARAM_LSM_CHANNEL';
prop2ParamMap('bidiPhaseAlignment') = 'PARAM_LSM_ALIGNMENT';
prop2ParamMap('inputChannelRange1') = 'PARAM_LSM_INPUTRANGE1';
prop2ParamMap('inputChannelRange2') = 'PARAM_LSM_INPUTRANGE2';
prop2ParamMap('inputChannelRange3') = 'PARAM_LSM_INPUTRANGE3';
prop2ParamMap('inputChannelRange4') = 'PARAM_LSM_INPUTRANGE4';
prop2ParamMap('scanMode') = 'PARAM_LSM_SCANMODE';
prop2ParamMap('averagingMode') = 'PARAM_LSM_AVERAGEMODE';
prop2ParamMap('averagingNumFrames') = 'PARAM_LSM_AVERAGENUM';
prop2ParamMap('clockSource') = 'PARAM_LSM_CLOCKSOURCE';
prop2ParamMap('clockRate') = 'PARAM_LSM_EXTERNALCLOCKRATE';
prop2ParamMap('triggerTimeout') = 'PARAM_TRIGGER_TIMEOUT_SEC';
prop2ParamMap('triggerFrameClockWithExtTrigger') = 'PARAM_ENABLE_FRAME_TRIGGER_WITH_HW_TRIG';
prop2ParamMap('areaMode') = 'PARAM_LSM_AREAMODE';
prop2ParamMap('offsetX') = 'PARAM_LSM_OFFSET_X';
prop2ParamMap('offsetY') = 'PARAM_LSM_OFFSET_Y';
prop2ParamMap('aspectRatioY') = 'PARAM_LSM_Y_AMPLITUDE_SCALER';
prop2ParamMap('flybackTimeLines') = 'PARAM_LSM_FLYBACK_CYCLE';
prop2ParamMap('frameTagEnable') = 'PARAM_LSM_APPEND_INDEX_TO_FRAME';
prop2ParamMap('dataMappingMode') = 'PARAM_LSM_DATAMAP_MODE';
prop2ParamMap('captureWithoutScanner') = 'PARAM_LSM_CAPTURE_WITHOUT_LINE_TRIGGER';
prop2ParamMap('touchParameter') = 'PARAM_LSM_FORCE_SETTINGS_UPDATE';
prop2ParamMap('bidiPhaseAlignmentCoarse1') = 'PARAM_LSM_TWO_WAY_ZONE_1';
prop2ParamMap('bidiPhaseAlignmentCoarse2') = 'PARAM_LSM_TWO_WAY_ZONE_2';
prop2ParamMap('bidiPhaseAlignmentCoarse3') = 'PARAM_LSM_TWO_WAY_ZONE_3';
prop2ParamMap('bidiPhaseAlignmentCoarse4') = 'PARAM_LSM_TWO_WAY_ZONE_4';
prop2ParamMap('bidiPhaseAlignmentCoarse5') = 'PARAM_LSM_TWO_WAY_ZONE_5';
prop2ParamMap('bidiPhaseAlignmentCoarse6') = 'PARAM_LSM_TWO_WAY_ZONE_6';
prop2ParamMap('bidiPhaseAlignmentCoarse7') = 'PARAM_LSM_TWO_WAY_ZONE_7';
prop2ParamMap('bidiPhaseAlignmentCoarse8') = 'PARAM_LSM_TWO_WAY_ZONE_8';
prop2ParamMap('bidiPhaseAlignmentCoarse9') = 'PARAM_LSM_TWO_WAY_ZONE_9';
prop2ParamMap('bidiPhaseAlignmentCoarse10') = 'PARAM_LSM_TWO_WAY_ZONE_10';
prop2ParamMap('bidiPhaseAlignmentCoarse11') = 'PARAM_LSM_TWO_WAY_ZONE_11';
prop2ParamMap('bidiPhaseAlignmentCoarse12') = 'PARAM_LSM_TWO_WAY_ZONE_12';
prop2ParamMap('bidiPhaseAlignmentCoarse13') = 'PARAM_LSM_TWO_WAY_ZONE_13';
prop2ParamMap('bidiPhaseAlignmentCoarse14') = 'PARAM_LSM_TWO_WAY_ZONE_14';
prop2ParamMap('bidiPhaseAlignmentCoarse15') = 'PARAM_LSM_TWO_WAY_ZONE_15';
prop2ParamMap('bidiPhaseAlignmentCoarse16') = 'PARAM_LSM_TWO_WAY_ZONE_16';
prop2ParamMap('bidiPhaseAlignmentCoarse17') = 'PARAM_LSM_TWO_WAY_ZONE_17';
prop2ParamMap('bidiPhaseAlignmentCoarse18') = 'PARAM_LSM_TWO_WAY_ZONE_18';
prop2ParamMap('bidiPhaseAlignmentCoarse19') = 'PARAM_LSM_TWO_WAY_ZONE_19';
prop2ParamMap('bidiPhaseAlignmentCoarse20') = 'PARAM_LSM_TWO_WAY_ZONE_20';
prop2ParamMap('bidiPhaseAlignmentCoarse21') = 'PARAM_LSM_TWO_WAY_ZONE_21';
prop2ParamMap('bidiPhaseAlignmentCoarse22') = 'PARAM_LSM_TWO_WAY_ZONE_22';
prop2ParamMap('bidiPhaseAlignmentCoarse23') = 'PARAM_LSM_TWO_WAY_ZONE_23';
prop2ParamMap('bidiPhaseAlignmentCoarse24') = 'PARAM_LSM_TWO_WAY_ZONE_24';
prop2ParamMap('bidiPhaseAlignmentCoarse25') = 'PARAM_LSM_TWO_WAY_ZONE_25';
prop2ParamMap('bidiPhaseAlignmentCoarse26') = 'PARAM_LSM_TWO_WAY_ZONE_26';
prop2ParamMap('bidiPhaseAlignmentCoarse27') = 'PARAM_LSM_TWO_WAY_ZONE_27';
prop2ParamMap('bidiPhaseAlignmentCoarse28') = 'PARAM_LSM_TWO_WAY_ZONE_28';
prop2ParamMap('bidiPhaseAlignmentCoarse29') = 'PARAM_LSM_TWO_WAY_ZONE_29';
prop2ParamMap('bidiPhaseAlignmentCoarse30') = 'PARAM_LSM_TWO_WAY_ZONE_30';
prop2ParamMap('bidiPhaseAlignmentCoarse31') = 'PARAM_LSM_TWO_WAY_ZONE_31';
prop2ParamMap('bidiPhaseAlignmentCoarse32') = 'PARAM_LSM_TWO_WAY_ZONE_32';
prop2ParamMap('bidiPhaseAlignmentCoarse33') = 'PARAM_LSM_TWO_WAY_ZONE_33';
prop2ParamMap('bidiPhaseAlignmentCoarse34') = 'PARAM_LSM_TWO_WAY_ZONE_34';
prop2ParamMap('bidiPhaseAlignmentCoarse35') = 'PARAM_LSM_TWO_WAY_ZONE_35';
prop2ParamMap('bidiPhaseAlignmentCoarse36') = 'PARAM_LSM_TWO_WAY_ZONE_36';
prop2ParamMap('bidiPhaseAlignmentCoarse37') = 'PARAM_LSM_TWO_WAY_ZONE_37';
prop2ParamMap('bidiPhaseAlignmentCoarse38') = 'PARAM_LSM_TWO_WAY_ZONE_38';
prop2ParamMap('bidiPhaseAlignmentCoarse39') = 'PARAM_LSM_TWO_WAY_ZONE_39';
prop2ParamMap('bidiPhaseAlignmentCoarse40') = 'PARAM_LSM_TWO_WAY_ZONE_40';
prop2ParamMap('bidiPhaseAlignmentCoarse41') = 'PARAM_LSM_TWO_WAY_ZONE_41';
prop2ParamMap('bidiPhaseAlignmentCoarse42') = 'PARAM_LSM_TWO_WAY_ZONE_42';
prop2ParamMap('bidiPhaseAlignmentCoarse43') = 'PARAM_LSM_TWO_WAY_ZONE_43';
prop2ParamMap('bidiPhaseAlignmentCoarse44') = 'PARAM_LSM_TWO_WAY_ZONE_44';
prop2ParamMap('bidiPhaseAlignmentCoarse45') = 'PARAM_LSM_TWO_WAY_ZONE_45';
prop2ParamMap('bidiPhaseAlignmentCoarse46') = 'PARAM_LSM_TWO_WAY_ZONE_46';
prop2ParamMap('bidiPhaseAlignmentCoarse47') = 'PARAM_LSM_TWO_WAY_ZONE_47';
prop2ParamMap('bidiPhaseAlignmentCoarse48') = 'PARAM_LSM_TWO_WAY_ZONE_48';
prop2ParamMap('bidiPhaseAlignmentCoarse49') = 'PARAM_LSM_TWO_WAY_ZONE_49';
prop2ParamMap('bidiPhaseAlignmentCoarse50') = 'PARAM_LSM_TWO_WAY_ZONE_50';
prop2ParamMap('bidiPhaseAlignmentCoarse51') = 'PARAM_LSM_TWO_WAY_ZONE_51';
prop2ParamMap('bidiPhaseAlignmentCoarse52') = 'PARAM_LSM_TWO_WAY_ZONE_52';
prop2ParamMap('bidiPhaseAlignmentCoarse53') = 'PARAM_LSM_TWO_WAY_ZONE_53';
prop2ParamMap('bidiPhaseAlignmentCoarse54') = 'PARAM_LSM_TWO_WAY_ZONE_54';
prop2ParamMap('bidiPhaseAlignmentCoarse55') = 'PARAM_LSM_TWO_WAY_ZONE_55';
prop2ParamMap('bidiPhaseAlignmentCoarse56') = 'PARAM_LSM_TWO_WAY_ZONE_56';
prop2ParamMap('bidiPhaseAlignmentCoarse57') = 'PARAM_LSM_TWO_WAY_ZONE_57';
prop2ParamMap('bidiPhaseAlignmentCoarse58') = 'PARAM_LSM_TWO_WAY_ZONE_58';
prop2ParamMap('bidiPhaseAlignmentCoarse59') = 'PARAM_LSM_TWO_WAY_ZONE_59';
prop2ParamMap('bidiPhaseAlignmentCoarse60') = 'PARAM_LSM_TWO_WAY_ZONE_60';
prop2ParamMap('bidiPhaseAlignmentCoarse61') = 'PARAM_LSM_TWO_WAY_ZONE_61';
prop2ParamMap('bidiPhaseAlignmentCoarse62') = 'PARAM_LSM_TWO_WAY_ZONE_62';
prop2ParamMap('bidiPhaseAlignmentCoarse63') = 'PARAM_LSM_TWO_WAY_ZONE_63';
prop2ParamMap('bidiPhaseAlignmentCoarse64') = 'PARAM_LSM_TWO_WAY_ZONE_64';
prop2ParamMap('bidiPhaseAlignmentCoarse65') = 'PARAM_LSM_TWO_WAY_ZONE_65';
prop2ParamMap('bidiPhaseAlignmentCoarse66') = 'PARAM_LSM_TWO_WAY_ZONE_66';
prop2ParamMap('bidiPhaseAlignmentCoarse67') = 'PARAM_LSM_TWO_WAY_ZONE_67';
prop2ParamMap('bidiPhaseAlignmentCoarse68') = 'PARAM_LSM_TWO_WAY_ZONE_68';
prop2ParamMap('bidiPhaseAlignmentCoarse69') = 'PARAM_LSM_TWO_WAY_ZONE_69';
prop2ParamMap('bidiPhaseAlignmentCoarse70') = 'PARAM_LSM_TWO_WAY_ZONE_70';
prop2ParamMap('bidiPhaseAlignmentCoarse71') = 'PARAM_LSM_TWO_WAY_ZONE_71';
prop2ParamMap('bidiPhaseAlignmentCoarse72') = 'PARAM_LSM_TWO_WAY_ZONE_72';
prop2ParamMap('bidiPhaseAlignmentCoarse73') = 'PARAM_LSM_TWO_WAY_ZONE_73';
prop2ParamMap('bidiPhaseAlignmentCoarse74') = 'PARAM_LSM_TWO_WAY_ZONE_74';
prop2ParamMap('bidiPhaseAlignmentCoarse75') = 'PARAM_LSM_TWO_WAY_ZONE_75';
prop2ParamMap('bidiPhaseAlignmentCoarse76') = 'PARAM_LSM_TWO_WAY_ZONE_76';
prop2ParamMap('bidiPhaseAlignmentCoarse77') = 'PARAM_LSM_TWO_WAY_ZONE_77';
prop2ParamMap('bidiPhaseAlignmentCoarse78') = 'PARAM_LSM_TWO_WAY_ZONE_78';
prop2ParamMap('bidiPhaseAlignmentCoarse79') = 'PARAM_LSM_TWO_WAY_ZONE_79';
prop2ParamMap('bidiPhaseAlignmentCoarse80') = 'PARAM_LSM_TWO_WAY_ZONE_80';
prop2ParamMap('bidiPhaseAlignmentCoarse81') = 'PARAM_LSM_TWO_WAY_ZONE_81';
prop2ParamMap('bidiPhaseAlignmentCoarse82') = 'PARAM_LSM_TWO_WAY_ZONE_82';
prop2ParamMap('bidiPhaseAlignmentCoarse83') = 'PARAM_LSM_TWO_WAY_ZONE_83';
prop2ParamMap('bidiPhaseAlignmentCoarse84') = 'PARAM_LSM_TWO_WAY_ZONE_84';
prop2ParamMap('bidiPhaseAlignmentCoarse85') = 'PARAM_LSM_TWO_WAY_ZONE_85';
prop2ParamMap('bidiPhaseAlignmentCoarse86') = 'PARAM_LSM_TWO_WAY_ZONE_86';
prop2ParamMap('bidiPhaseAlignmentCoarse87') = 'PARAM_LSM_TWO_WAY_ZONE_87';
prop2ParamMap('bidiPhaseAlignmentCoarse88') = 'PARAM_LSM_TWO_WAY_ZONE_88';
prop2ParamMap('bidiPhaseAlignmentCoarse89') = 'PARAM_LSM_TWO_WAY_ZONE_89';
prop2ParamMap('bidiPhaseAlignmentCoarse90') = 'PARAM_LSM_TWO_WAY_ZONE_90';
prop2ParamMap('bidiPhaseAlignmentCoarse91') = 'PARAM_LSM_TWO_WAY_ZONE_91';
prop2ParamMap('bidiPhaseAlignmentCoarse92') = 'PARAM_LSM_TWO_WAY_ZONE_92';
prop2ParamMap('bidiPhaseAlignmentCoarse93') = 'PARAM_LSM_TWO_WAY_ZONE_93';
prop2ParamMap('bidiPhaseAlignmentCoarse94') = 'PARAM_LSM_TWO_WAY_ZONE_94';
prop2ParamMap('bidiPhaseAlignmentCoarse95') = 'PARAM_LSM_TWO_WAY_ZONE_95';
prop2ParamMap('bidiPhaseAlignmentCoarse96') = 'PARAM_LSM_TWO_WAY_ZONE_96';
prop2ParamMap('bidiPhaseAlignmentCoarse97') = 'PARAM_LSM_TWO_WAY_ZONE_97';
prop2ParamMap('bidiPhaseAlignmentCoarse98') = 'PARAM_LSM_TWO_WAY_ZONE_98';
prop2ParamMap('bidiPhaseAlignmentCoarse99') = 'PARAM_LSM_TWO_WAY_ZONE_99';
prop2ParamMap('bidiPhaseAlignmentCoarse100') = 'PARAM_LSM_TWO_WAY_ZONE_100';
prop2ParamMap('bidiPhaseAlignmentCoarse101') = 'PARAM_LSM_TWO_WAY_ZONE_101';
prop2ParamMap('bidiPhaseAlignmentCoarse102') = 'PARAM_LSM_TWO_WAY_ZONE_102';
prop2ParamMap('bidiPhaseAlignmentCoarse103') = 'PARAM_LSM_TWO_WAY_ZONE_103';
prop2ParamMap('bidiPhaseAlignmentCoarse104') = 'PARAM_LSM_TWO_WAY_ZONE_104';
prop2ParamMap('bidiPhaseAlignmentCoarse105') = 'PARAM_LSM_TWO_WAY_ZONE_105';
prop2ParamMap('bidiPhaseAlignmentCoarse106') = 'PARAM_LSM_TWO_WAY_ZONE_106';
prop2ParamMap('bidiPhaseAlignmentCoarse107') = 'PARAM_LSM_TWO_WAY_ZONE_107';
prop2ParamMap('bidiPhaseAlignmentCoarse108') = 'PARAM_LSM_TWO_WAY_ZONE_108';
prop2ParamMap('bidiPhaseAlignmentCoarse109') = 'PARAM_LSM_TWO_WAY_ZONE_109';
prop2ParamMap('bidiPhaseAlignmentCoarse110') = 'PARAM_LSM_TWO_WAY_ZONE_110';
prop2ParamMap('bidiPhaseAlignmentCoarse111') = 'PARAM_LSM_TWO_WAY_ZONE_111';
prop2ParamMap('bidiPhaseAlignmentCoarse112') = 'PARAM_LSM_TWO_WAY_ZONE_112';
prop2ParamMap('bidiPhaseAlignmentCoarse113') = 'PARAM_LSM_TWO_WAY_ZONE_113';
prop2ParamMap('bidiPhaseAlignmentCoarse114') = 'PARAM_LSM_TWO_WAY_ZONE_114';
prop2ParamMap('bidiPhaseAlignmentCoarse115') = 'PARAM_LSM_TWO_WAY_ZONE_115';
prop2ParamMap('bidiPhaseAlignmentCoarse116') = 'PARAM_LSM_TWO_WAY_ZONE_116';
prop2ParamMap('bidiPhaseAlignmentCoarse117') = 'PARAM_LSM_TWO_WAY_ZONE_117';
prop2ParamMap('bidiPhaseAlignmentCoarse118') = 'PARAM_LSM_TWO_WAY_ZONE_118';
prop2ParamMap('bidiPhaseAlignmentCoarse119') = 'PARAM_LSM_TWO_WAY_ZONE_119';
prop2ParamMap('bidiPhaseAlignmentCoarse120') = 'PARAM_LSM_TWO_WAY_ZONE_120';
prop2ParamMap('bidiPhaseAlignmentCoarse121') = 'PARAM_LSM_TWO_WAY_ZONE_121';
prop2ParamMap('bidiPhaseAlignmentCoarse122') = 'PARAM_LSM_TWO_WAY_ZONE_122';
prop2ParamMap('bidiPhaseAlignmentCoarse123') = 'PARAM_LSM_TWO_WAY_ZONE_123';
prop2ParamMap('bidiPhaseAlignmentCoarse124') = 'PARAM_LSM_TWO_WAY_ZONE_124';
prop2ParamMap('bidiPhaseAlignmentCoarse125') = 'PARAM_LSM_TWO_WAY_ZONE_125';
prop2ParamMap('bidiPhaseAlignmentCoarse126') = 'PARAM_LSM_TWO_WAY_ZONE_126';
prop2ParamMap('bidiPhaseAlignmentCoarse127') = 'PARAM_LSM_TWO_WAY_ZONE_127';
prop2ParamMap('bidiPhaseAlignmentCoarse128') = 'PARAM_LSM_TWO_WAY_ZONE_128';
prop2ParamMap('bidiPhaseAlignmentCoarse129') = 'PARAM_LSM_TWO_WAY_ZONE_129';
prop2ParamMap('bidiPhaseAlignmentCoarse130') = 'PARAM_LSM_TWO_WAY_ZONE_130';
prop2ParamMap('bidiPhaseAlignmentCoarse131') = 'PARAM_LSM_TWO_WAY_ZONE_131';
prop2ParamMap('bidiPhaseAlignmentCoarse132') = 'PARAM_LSM_TWO_WAY_ZONE_132';
prop2ParamMap('bidiPhaseAlignmentCoarse133') = 'PARAM_LSM_TWO_WAY_ZONE_133';
prop2ParamMap('bidiPhaseAlignmentCoarse134') = 'PARAM_LSM_TWO_WAY_ZONE_134';
prop2ParamMap('bidiPhaseAlignmentCoarse135') = 'PARAM_LSM_TWO_WAY_ZONE_135';
prop2ParamMap('bidiPhaseAlignmentCoarse136') = 'PARAM_LSM_TWO_WAY_ZONE_136';
prop2ParamMap('bidiPhaseAlignmentCoarse137') = 'PARAM_LSM_TWO_WAY_ZONE_137';
prop2ParamMap('bidiPhaseAlignmentCoarse138') = 'PARAM_LSM_TWO_WAY_ZONE_138';
prop2ParamMap('bidiPhaseAlignmentCoarse139') = 'PARAM_LSM_TWO_WAY_ZONE_139';
prop2ParamMap('bidiPhaseAlignmentCoarse140') = 'PARAM_LSM_TWO_WAY_ZONE_140';
prop2ParamMap('bidiPhaseAlignmentCoarse141') = 'PARAM_LSM_TWO_WAY_ZONE_141';
prop2ParamMap('bidiPhaseAlignmentCoarse142') = 'PARAM_LSM_TWO_WAY_ZONE_142';
prop2ParamMap('bidiPhaseAlignmentCoarse143') = 'PARAM_LSM_TWO_WAY_ZONE_143';
prop2ParamMap('bidiPhaseAlignmentCoarse144') = 'PARAM_LSM_TWO_WAY_ZONE_144';
prop2ParamMap('bidiPhaseAlignmentCoarse145') = 'PARAM_LSM_TWO_WAY_ZONE_145';
prop2ParamMap('bidiPhaseAlignmentCoarse146') = 'PARAM_LSM_TWO_WAY_ZONE_146';
prop2ParamMap('bidiPhaseAlignmentCoarse147') = 'PARAM_LSM_TWO_WAY_ZONE_147';
prop2ParamMap('bidiPhaseAlignmentCoarse148') = 'PARAM_LSM_TWO_WAY_ZONE_148';
prop2ParamMap('bidiPhaseAlignmentCoarse149') = 'PARAM_LSM_TWO_WAY_ZONE_149';
prop2ParamMap('bidiPhaseAlignmentCoarse150') = 'PARAM_LSM_TWO_WAY_ZONE_150';
prop2ParamMap('bidiPhaseAlignmentCoarse151') = 'PARAM_LSM_TWO_WAY_ZONE_151';
prop2ParamMap('bidiPhaseAlignmentCoarse152') = 'PARAM_LSM_TWO_WAY_ZONE_152';
prop2ParamMap('bidiPhaseAlignmentCoarse153') = 'PARAM_LSM_TWO_WAY_ZONE_153';
prop2ParamMap('bidiPhaseAlignmentCoarse154') = 'PARAM_LSM_TWO_WAY_ZONE_154';
prop2ParamMap('bidiPhaseAlignmentCoarse155') = 'PARAM_LSM_TWO_WAY_ZONE_155';
prop2ParamMap('bidiPhaseAlignmentCoarse156') = 'PARAM_LSM_TWO_WAY_ZONE_156';
prop2ParamMap('bidiPhaseAlignmentCoarse157') = 'PARAM_LSM_TWO_WAY_ZONE_157';
prop2ParamMap('bidiPhaseAlignmentCoarse158') = 'PARAM_LSM_TWO_WAY_ZONE_158';
prop2ParamMap('bidiPhaseAlignmentCoarse159') = 'PARAM_LSM_TWO_WAY_ZONE_159';
prop2ParamMap('bidiPhaseAlignmentCoarse160') = 'PARAM_LSM_TWO_WAY_ZONE_160';
prop2ParamMap('bidiPhaseAlignmentCoarse161') = 'PARAM_LSM_TWO_WAY_ZONE_161';
prop2ParamMap('bidiPhaseAlignmentCoarse162') = 'PARAM_LSM_TWO_WAY_ZONE_162';
prop2ParamMap('bidiPhaseAlignmentCoarse163') = 'PARAM_LSM_TWO_WAY_ZONE_163';
prop2ParamMap('bidiPhaseAlignmentCoarse164') = 'PARAM_LSM_TWO_WAY_ZONE_164';
prop2ParamMap('bidiPhaseAlignmentCoarse165') = 'PARAM_LSM_TWO_WAY_ZONE_165';
prop2ParamMap('bidiPhaseAlignmentCoarse166') = 'PARAM_LSM_TWO_WAY_ZONE_166';
prop2ParamMap('bidiPhaseAlignmentCoarse167') = 'PARAM_LSM_TWO_WAY_ZONE_167';
prop2ParamMap('bidiPhaseAlignmentCoarse168') = 'PARAM_LSM_TWO_WAY_ZONE_168';
prop2ParamMap('bidiPhaseAlignmentCoarse169') = 'PARAM_LSM_TWO_WAY_ZONE_169';
prop2ParamMap('bidiPhaseAlignmentCoarse170') = 'PARAM_LSM_TWO_WAY_ZONE_170';
prop2ParamMap('bidiPhaseAlignmentCoarse171') = 'PARAM_LSM_TWO_WAY_ZONE_171';
prop2ParamMap('bidiPhaseAlignmentCoarse172') = 'PARAM_LSM_TWO_WAY_ZONE_172';
prop2ParamMap('bidiPhaseAlignmentCoarse173') = 'PARAM_LSM_TWO_WAY_ZONE_173';
prop2ParamMap('bidiPhaseAlignmentCoarse174') = 'PARAM_LSM_TWO_WAY_ZONE_174';
prop2ParamMap('bidiPhaseAlignmentCoarse175') = 'PARAM_LSM_TWO_WAY_ZONE_175';
prop2ParamMap('bidiPhaseAlignmentCoarse176') = 'PARAM_LSM_TWO_WAY_ZONE_176';
prop2ParamMap('bidiPhaseAlignmentCoarse177') = 'PARAM_LSM_TWO_WAY_ZONE_177';
prop2ParamMap('bidiPhaseAlignmentCoarse178') = 'PARAM_LSM_TWO_WAY_ZONE_178';
prop2ParamMap('bidiPhaseAlignmentCoarse179') = 'PARAM_LSM_TWO_WAY_ZONE_179';
prop2ParamMap('bidiPhaseAlignmentCoarse180') = 'PARAM_LSM_TWO_WAY_ZONE_180';
prop2ParamMap('bidiPhaseAlignmentCoarse181') = 'PARAM_LSM_TWO_WAY_ZONE_181';
prop2ParamMap('bidiPhaseAlignmentCoarse182') = 'PARAM_LSM_TWO_WAY_ZONE_182';
prop2ParamMap('bidiPhaseAlignmentCoarse183') = 'PARAM_LSM_TWO_WAY_ZONE_183';
prop2ParamMap('bidiPhaseAlignmentCoarse184') = 'PARAM_LSM_TWO_WAY_ZONE_184';
prop2ParamMap('bidiPhaseAlignmentCoarse185') = 'PARAM_LSM_TWO_WAY_ZONE_185';
prop2ParamMap('bidiPhaseAlignmentCoarse186') = 'PARAM_LSM_TWO_WAY_ZONE_186';
prop2ParamMap('bidiPhaseAlignmentCoarse187') = 'PARAM_LSM_TWO_WAY_ZONE_187';
prop2ParamMap('bidiPhaseAlignmentCoarse188') = 'PARAM_LSM_TWO_WAY_ZONE_188';
prop2ParamMap('bidiPhaseAlignmentCoarse189') = 'PARAM_LSM_TWO_WAY_ZONE_189';
prop2ParamMap('bidiPhaseAlignmentCoarse190') = 'PARAM_LSM_TWO_WAY_ZONE_190';
prop2ParamMap('bidiPhaseAlignmentCoarse191') = 'PARAM_LSM_TWO_WAY_ZONE_191';
prop2ParamMap('bidiPhaseAlignmentCoarse192') = 'PARAM_LSM_TWO_WAY_ZONE_192';
prop2ParamMap('bidiPhaseAlignmentCoarse193') = 'PARAM_LSM_TWO_WAY_ZONE_193';
prop2ParamMap('bidiPhaseAlignmentCoarse194') = 'PARAM_LSM_TWO_WAY_ZONE_194';
prop2ParamMap('bidiPhaseAlignmentCoarse195') = 'PARAM_LSM_TWO_WAY_ZONE_195';
prop2ParamMap('bidiPhaseAlignmentCoarse196') = 'PARAM_LSM_TWO_WAY_ZONE_196';
prop2ParamMap('bidiPhaseAlignmentCoarse197') = 'PARAM_LSM_TWO_WAY_ZONE_197';
prop2ParamMap('bidiPhaseAlignmentCoarse198') = 'PARAM_LSM_TWO_WAY_ZONE_198';
prop2ParamMap('bidiPhaseAlignmentCoarse199') = 'PARAM_LSM_TWO_WAY_ZONE_199';
prop2ParamMap('bidiPhaseAlignmentCoarse200') = 'PARAM_LSM_TWO_WAY_ZONE_200';
prop2ParamMap('bidiPhaseAlignmentCoarse201') = 'PARAM_LSM_TWO_WAY_ZONE_201';
prop2ParamMap('bidiPhaseAlignmentCoarse202') = 'PARAM_LSM_TWO_WAY_ZONE_202';
prop2ParamMap('bidiPhaseAlignmentCoarse203') = 'PARAM_LSM_TWO_WAY_ZONE_203';
prop2ParamMap('bidiPhaseAlignmentCoarse204') = 'PARAM_LSM_TWO_WAY_ZONE_204';
prop2ParamMap('bidiPhaseAlignmentCoarse205') = 'PARAM_LSM_TWO_WAY_ZONE_205';
prop2ParamMap('bidiPhaseAlignmentCoarse206') = 'PARAM_LSM_TWO_WAY_ZONE_206';
prop2ParamMap('bidiPhaseAlignmentCoarse207') = 'PARAM_LSM_TWO_WAY_ZONE_207';
prop2ParamMap('bidiPhaseAlignmentCoarse208') = 'PARAM_LSM_TWO_WAY_ZONE_208';
prop2ParamMap('bidiPhaseAlignmentCoarse209') = 'PARAM_LSM_TWO_WAY_ZONE_209';
prop2ParamMap('bidiPhaseAlignmentCoarse210') = 'PARAM_LSM_TWO_WAY_ZONE_210';
prop2ParamMap('bidiPhaseAlignmentCoarse211') = 'PARAM_LSM_TWO_WAY_ZONE_211';
prop2ParamMap('bidiPhaseAlignmentCoarse212') = 'PARAM_LSM_TWO_WAY_ZONE_212';
prop2ParamMap('bidiPhaseAlignmentCoarse213') = 'PARAM_LSM_TWO_WAY_ZONE_213';
prop2ParamMap('bidiPhaseAlignmentCoarse214') = 'PARAM_LSM_TWO_WAY_ZONE_214';
prop2ParamMap('bidiPhaseAlignmentCoarse215') = 'PARAM_LSM_TWO_WAY_ZONE_215';
prop2ParamMap('bidiPhaseAlignmentCoarse216') = 'PARAM_LSM_TWO_WAY_ZONE_216';
prop2ParamMap('bidiPhaseAlignmentCoarse217') = 'PARAM_LSM_TWO_WAY_ZONE_217';
prop2ParamMap('bidiPhaseAlignmentCoarse218') = 'PARAM_LSM_TWO_WAY_ZONE_218';
prop2ParamMap('bidiPhaseAlignmentCoarse219') = 'PARAM_LSM_TWO_WAY_ZONE_219';
prop2ParamMap('bidiPhaseAlignmentCoarse220') = 'PARAM_LSM_TWO_WAY_ZONE_220';
prop2ParamMap('bidiPhaseAlignmentCoarse221') = 'PARAM_LSM_TWO_WAY_ZONE_221';
prop2ParamMap('bidiPhaseAlignmentCoarse222') = 'PARAM_LSM_TWO_WAY_ZONE_222';
prop2ParamMap('bidiPhaseAlignmentCoarse223') = 'PARAM_LSM_TWO_WAY_ZONE_223';
prop2ParamMap('bidiPhaseAlignmentCoarse224') = 'PARAM_LSM_TWO_WAY_ZONE_224';
prop2ParamMap('bidiPhaseAlignmentCoarse225') = 'PARAM_LSM_TWO_WAY_ZONE_225';
prop2ParamMap('bidiPhaseAlignmentCoarse226') = 'PARAM_LSM_TWO_WAY_ZONE_226';
prop2ParamMap('bidiPhaseAlignmentCoarse227') = 'PARAM_LSM_TWO_WAY_ZONE_227';
prop2ParamMap('bidiPhaseAlignmentCoarse228') = 'PARAM_LSM_TWO_WAY_ZONE_228';
prop2ParamMap('bidiPhaseAlignmentCoarse229') = 'PARAM_LSM_TWO_WAY_ZONE_229';
prop2ParamMap('bidiPhaseAlignmentCoarse230') = 'PARAM_LSM_TWO_WAY_ZONE_230';
prop2ParamMap('bidiPhaseAlignmentCoarse231') = 'PARAM_LSM_TWO_WAY_ZONE_231';
prop2ParamMap('bidiPhaseAlignmentCoarse232') = 'PARAM_LSM_TWO_WAY_ZONE_232';
prop2ParamMap('bidiPhaseAlignmentCoarse233') = 'PARAM_LSM_TWO_WAY_ZONE_233';
prop2ParamMap('bidiPhaseAlignmentCoarse234') = 'PARAM_LSM_TWO_WAY_ZONE_234';
prop2ParamMap('bidiPhaseAlignmentCoarse235') = 'PARAM_LSM_TWO_WAY_ZONE_235';
prop2ParamMap('bidiPhaseAlignmentCoarse236') = 'PARAM_LSM_TWO_WAY_ZONE_236';
prop2ParamMap('bidiPhaseAlignmentCoarse237') = 'PARAM_LSM_TWO_WAY_ZONE_237';
prop2ParamMap('bidiPhaseAlignmentCoarse238') = 'PARAM_LSM_TWO_WAY_ZONE_238';
prop2ParamMap('bidiPhaseAlignmentCoarse239') = 'PARAM_LSM_TWO_WAY_ZONE_239';
prop2ParamMap('bidiPhaseAlignmentCoarse240') = 'PARAM_LSM_TWO_WAY_ZONE_240';
prop2ParamMap('bidiPhaseAlignmentCoarse241') = 'PARAM_LSM_TWO_WAY_ZONE_241';
prop2ParamMap('bidiPhaseAlignmentCoarse242') = 'PARAM_LSM_TWO_WAY_ZONE_242';
prop2ParamMap('bidiPhaseAlignmentCoarse243') = 'PARAM_LSM_TWO_WAY_ZONE_243';
prop2ParamMap('bidiPhaseAlignmentCoarse244') = 'PARAM_LSM_TWO_WAY_ZONE_244';
prop2ParamMap('bidiPhaseAlignmentCoarse245') = 'PARAM_LSM_TWO_WAY_ZONE_245';
prop2ParamMap('bidiPhaseAlignmentCoarse246') = 'PARAM_LSM_TWO_WAY_ZONE_246';
prop2ParamMap('bidiPhaseAlignmentCoarse247') = 'PARAM_LSM_TWO_WAY_ZONE_247';
prop2ParamMap('bidiPhaseAlignmentCoarse248') = 'PARAM_LSM_TWO_WAY_ZONE_248';
prop2ParamMap('bidiPhaseAlignmentCoarse249') = 'PARAM_LSM_TWO_WAY_ZONE_249';
prop2ParamMap('bidiPhaseAlignmentCoarse250') = 'PARAM_LSM_TWO_WAY_ZONE_250';
prop2ParamMap('bidiPhaseAlignmentCoarse251') = 'PARAM_LSM_TWO_WAY_ZONE_251';

prop2ParamMap('galvoEnable') = 'PARAM_LSM_GALVO_ENABLE';

end
