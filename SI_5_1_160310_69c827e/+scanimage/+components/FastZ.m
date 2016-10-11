classdef FastZ < scanimage.interfaces.Component & most.HasMachineDataFile
    %% USER PROPS
    properties (SetObservable)
        enable = false;                 %Boolean, when true, FastZ is enabled.
        numVolumes=1;                   %Number of total FastZ volumes to capture for a given Acq.
        settlingTime = 0;               %Time, in seconds, for axial position/ramp to settle.
        discardFlybackFrames = false;   %Logical indicating whether to discard frames during fastZ scanner flyback
        volumePeriodAdjustment = -6e-4; %Time, in s, to add to the nominal volume period, when determining fastZ sawtooth period used for volume imaging
        acquisitionDelay;               %Acquisition delay, in seconds, of fastZScanner. Value is exactly 1/2 the settlingTime.
        waveformType = 'sawtooth';      %Can be either 'waveform' or 'step'
        useArbitraryZs = false;         %In step/settle mode, use z's entered by user rather than num slices/steps per slice
        userZs = 0;                     %In step/settle mode, the arbitrary z series to use
    end
    
    properties (SetObservable,SetAccess=?scanimage.interfaces.Class,Transient)
        volumesDone = 0;                %Integer, incremented every time a FastZ Volume is acquired. Only incremented when grabbing FastZ volumes. Excluded from frame header
    end
    
    properties (SetObservable,SetAccess=private)
        numFramesPerVolume;             %Number of frames per FastZ volume for current Acq & FastZ settings (includes flyback Frames)
        positionAbsolute;               %Used by parent object to get hAOTask.positionAbsolute
        period;                         %Time specification in seconds. Co-varies with slicesPerAcq/stackZStepSize.
        fillFraction;                   %Fraction of frames in acquisition stream during fastZ imaging
    end
    
    properties (SetObservable,SetAccess=private,Transient)
        extFrameClockTerminal;
    end
    
    properties (Dependent,SetObservable)
        numDiscardFlybackFrames;        %Number of discarded frames for each period
    end
    
    properties (Dependent,Transient)
        positionTarget;                 %Used by parent object to get hAOTask.positionTarget
        secondMotor;
    end
    
    properties (SetAccess=private,Transient)
        homePosition;                   %Cache of the fastZ controller's position at start of acquisition mode, which should be restored at conclusion of acquisition mode
    end
    
    %% INTERNAL PROPS
    properties (Hidden,SetAccess=private)
        aoOutputRate;                   %Automatically set to maximum AO output rate supported by DAQ device
        aoData;                         
        requireAO = false;
        useAOControl = true;            %Logical indicating whether to use AO control of fastZ hardware during FastZ operations
        extFrameClockTerminal_;
        extFrameClockImgSys_;
        simulatedDevice = false;        %Logical indicating whether or not the configured FastZ device is simulated.
        
        hDevice;                        %Handle to FastZ hardware, may be a LSC object or a PI motion controller
        hAOTask;                        %Handle to DAQmx AO Task used for FastZ sweep/step control
    end
    
    %%% ABSTRACT PROPERTY REALIZATIONS (most.HasMachineDataFile)
    properties (Constant, Hidden)
        %Value-Required properties
        mdfClassName = mfilename('class');
        mdfHeading = 'FastZ';
        
        %Value-Optional properties
        mdfDependsOnClasses; %#ok<MCCPI>
        mdfDirectProp;       %#ok<MCCPI>
        mdfPropPrefix;       %#ok<MCCPI>
    end
    
    %%% ABSTRACT PROPERTY REALIZATION (most.Model)
    properties (Hidden, SetAccess=protected)
        mdlPropAttributes = ziniInitPropAttributes();
        mdlHeaderExcludeProps = {'volumesDone'}
    end
    
    %%% ABSTRACT PROPERTY REALIZATION (scanimage.interfaces.Component)
    properties (SetAccess = protected, Hidden)
        numInstances = 0;
    end
    
    properties (Constant, Hidden)
        COMPONENT_NAME = 'FastZ';                                       % [char array] short name describing functionality of component e.g. 'Beams' or 'FastZ'
        PROP_TRUE_LIVE_UPDATE = {};                                     % Cell array of strings specifying properties that can be set while the component is active
        PROP_FOCUS_TRUE_LIVE_UPDATE = {};                               % Cell array of strings specifying properties that can be set while focusing
        DENY_PROP_LIVE_UPDATE = {'enable','numVolumes'};                % Cell array of strings specifying properties for which a live update is denied (during acqState = Focus)
        FUNC_TRUE_LIVE_EXECUTION = {'setHome','resetHome',...           % Cell array of strings specifying functions that can be executed while the component is active
            'goPark','goHome'};
        FUNC_FOCUS_TRUE_LIVE_EXECUTION = {};                            % Cell array of strings specifying functions that can be executed while focusing
        DENY_FUNC_LIVE_EXECUTION = {};                                  % Cell array of strings specifying functions for which a live execution is denied (during acqState = Focus)
    end
    
    %% LIFECYCLE
    methods (Hidden)
        function obj = FastZ(hSI)
            obj = obj@scanimage.interfaces.Component(hSI,[]);
        end
        
        function delete(obj)
            if ~isempty(obj.hAOTask)
                obj.hAOTask.stop();
                delete(obj.hAOTask);
                clear obj.hAOTask;
            end
        end
    end
    
    %% PROP ACCESS
    methods
        function set.enable(obj,val)
            val = obj.validatePropArg('enable',val);
                
            if (~val || obj.numInstances) && obj.componentUpdateProperty('enable',val)
                obj.enable = val;
                obj.hSI.hStackManager.updateZSeries();
                
                if val
                    obj.hSI.hStackManager.framesPerSlice = 1;
                end
            end
        end
        
        function set.numVolumes(obj,val)
            if ~isinf(val)
                val = obj.validatePropArg('numVolumes',val);
            else
                val = abs(val); % in case someone tried to set the value to -Inf
            end
            
            if obj.componentUpdateProperty('numVolumes',val) 
                obj.numVolumes = val;
            end
        end
        
        function val = get.numFramesPerVolume(obj)
            if obj.enable
            	val = obj.hSI.hStackManager.slicesPerAcq + obj.numDiscardFlybackFrames;
            else
                val = [];
            end
        end
        
        function val = get.secondMotor(obj)
            val = strcmpi(obj.mdfData.fastZControllerType,'useMotor2');
        end
        
        function val = get.positionAbsolute(obj)
            if ~isempty(obj.hDevice)
                if obj.secondMotor
                    val = obj.hDevice.positionAbsolute(end);
                else
                    val = obj.hDevice.positionAbsolute(1);
                end
            else
                val = NaN;
            end
        end
        
        function set.numDiscardFlybackFrames(obj,val)
            obj.mdlDummySetProp(val,'numDiscardFlybackFrames');
            obj.hSI.hDisplay.updateFrameBatchProps();
        end
        
        function val = get.numDiscardFlybackFrames(obj)
            if obj.discardFlybackFrames && obj.enable && strcmp(obj.waveformType, 'sawtooth') && (obj.numVolumes > 1)
                %TODO: Tighten up these computations a bit to deal with edge cases
                %TODO: Could account for maximum slew rate as well, at least when 'velocity' property is available                
                
                settlingNumSamples = round(obj.aoOutputRate * obj.settlingTime);
                frameNumSamples = obj.aoOutputRate * obj.hSI.hRoiManager.scanFramePeriod;
                
                val = ceil(settlingNumSamples/frameNumSamples);
                
                if isinf(val)
                    val = 0;
                end
            else
                val = 0;
            end
        end
        
        function val = get.positionTarget(obj)
            if ~isempty(obj.hDevice)
                if obj.secondMotor
                    val = obj.hDevice.positionTarget(end);
                else
                    val = obj.hDevice.positionTarget(1);
                end
            else
                val = NaN;
            end
        end
        
        function set.positionTarget(obj,v)
            if obj.secondMotor
                obj.hDevice.moveCompleteAbsolute([nan nan v]);
            else
                obj.hDevice.moveCompleteAbsolute([v nan nan]);
            end
        end
        
        function set.acquisitionDelay(obj,val)
            if obj.componentUpdateProperty('acquisitionDelay',val)
                obj.settlingTime = 2 * val; %#ok<MCSUP>
            end
        end
        
        function val = get.acquisitionDelay(obj)
            val = obj.settlingTime / 2;
        end
        
        function set.settlingTime(obj,val)
            val = obj.validatePropArg('settlingTime',val);
            if obj.componentUpdateProperty('settlingTime',val)
                obj.settlingTime = val;
            end
        end
        
        function set.discardFlybackFrames(obj,val)
            if obj.componentUpdateProperty('discardFlybackFrames',val)
                obj.discardFlybackFrames = val;
            end
        end
        
        function set.volumePeriodAdjustment(obj,val)
            if obj.componentUpdateProperty('volumePeriodAdjustment',val)
                obj.volumePeriodAdjustment = val;
            end
        end
        
        function set.waveformType(obj,val)
            if obj.componentUpdateProperty('waveformType',val)
                assert(ismember(val,{'sawtooth' 'step'}), 'Invalid selection for waveform tpye. Must be either ''sawtooth'' or ''step''.');
                obj.waveformType = val;
                obj.hSI.hStackManager.updateZSeries();
            end
        end
        
        function set.userZs(obj,v)
            if obj.componentUpdateProperty('userZs',v)
                if isempty(v)
                    v = 0;
                end
                obj.userZs = v;
                obj.hSI.hStackManager.updateZSeries();
            end
        end
        
        function set.useArbitraryZs(obj,v)
            if obj.componentUpdateProperty('useArbitraryZs',v)
                obj.useArbitraryZs = v;
                obj.hSI.hStackManager.updateZSeries();
            end
        end
        
        function set.useAOControl(obj,val)
            if obj.componentUpdateProperty('useAOControl',val)
                
                if obj.requireAO %#ok<MCSUP>
                    val = true;
                end
                obj.useAOControl = val;
            end
        end
        
        function val = get.extFrameClockTerminal(obj)
            % This routine configures the start trigger for hTask
            % it first tries to connect the start trigger to the internal
            % beamsclock output of Scan2D. If this route fails, it uses the
            % external trigger terminal configured in the MDF
            
            if (isempty(obj.extFrameClockTerminal_) || ~strcmp(obj.extFrameClockImgSys_,obj.hSI.imagingSystem))...
                    && ~isempty(obj.hAOTask)
                try
                    % Try internal routing
                    internalTrigTerm = obj.hSI.hScan2D.trigFrameClkOutInternalTerm;
                    obj.hAOTask.cfgDigEdgeStartTrig(internalTrigTerm);
                    obj.hAOTask.control('DAQmx_Val_Task_Reserve'); % if no internal route is available, this call will throw an error
                    obj.hAOTask.control('DAQmx_Val_Task_Unreserve');
                    
                    val = internalTrigTerm;
                    % fprintf('FastZ: internal trigger route found: %s\n',val);
                catch ME
                    % Error -89125 is expected: No registered trigger lines could be found between the devices in the route.
                    % Error -89139 is expected: There are no shared trigger lines between the two devices which are acceptable to both devices.
                    if isempty(strfind(ME.message, '-89125')) && isempty(strfind(ME.message, '-89139')) % filter error messages
                        rethrow(ME)
                    end
                    
                    % No internal route available - use MDF settings
					try
                        obj.zprvMDFVerify('frameClockIn',{{'char'},{'vector'}},[]);
                    catch ME
                        fprintf(2,'FastZ cannot synchronize to scanning system. See error message below:\n\n');
                        rethrow(ME);
                    end
                    
                    val = obj.mdfData.frameClockIn;
                    % fprintf('FastZ: no internal trigger route found. Using MDF settings: %s\n',val);
                end
                obj.extFrameClockTerminal_ = val;
                obj.extFrameClockImgSys_ = obj.hSI.imagingSystem;
                
            else
                val = obj.extFrameClockTerminal_;
            end
        end
    end
    
    %% USER METHODS
    methods
        function setHome(obj,val)
            if obj.componentExecuteFunction('setHome',val)
                %set homePosition.
                obj.homePosition = val;
            end
        end
        
        function resetHome(obj)
            if obj.componentExecuteFunction('resetHome')
                %Reset fastZ positions
                obj.homePosition = [];
            end
        end
        
        function goHome(obj)
            if obj.componentExecuteFunction('goHome')
                %Go to home fastZ position, as applicable
                if ~isempty(obj.homePosition)
                    obj.goTo(obj.homePosition);
                end
            end
        end
        
        function goPark(obj)
            if obj.componentExecuteFunction('goPark')
                disp('Moving Fast Z Stage to 0.0...');
                obj.goTo(0);
            end
        end
    end
    
    %% FRIEND METHODS
    
    
    %% INTERNAL METHODS
    methods (Hidden, Access=private)
        
        function goTo(obj,v)
            if obj.secondMotor
                obj.hDevice.moveCompleteAbsolute([nan nan v]);
            else
                obj.hDevice.moveCompleteAbsolute([v nan nan]);
            end
        end
        
        function updateTaskConfiguration(obj)
            assert(~isempty(obj.aoData));
            
            %Detect if command is outside allowable voltage range
            if max(obj.aoData.volts(:)) > obj.hDevice.hLSC.analogCmdOutputRange(2)
                maxClamp =  obj.hDevice.hLSC.analogCmdOutputRange(2);
                obj.aoData.volts(obj.aoData.volts > maxClamp) = maxClamp;
                most.idioms.dispError('WARNING: Computed FastZ AO data exceeds maximum voltage of AO channel (%g). Full range of specified scan will not be achieved.\n',maxClamp);
            end
            
            if min(obj.aoData.volts(:)) < obj.hDevice.hLSC.analogCmdOutputRange(1)
                minClamp =  obj.hDevice.hLSC.analogCmdOutputRange(1);
                obj.aoData.volts(obj.aoData.volts < minClamp) = minClamp;
                most.idioms.dispError('WARNING: Computed FastZ AO data falls below minimum voltage of AO channel (%g). Full range of specified scan will not be achieved.\n',minClamp);
            end

            %Update AO Buffer
            obj.hAOTask.control('DAQmx_Val_Task_Unreserve'); %Flush any previous data in the buffer
            obj.hAOTask.cfgDigEdgeStartTrig(obj.extFrameClockTerminal, obj.aoData.triggerEdge);
            obj.hAOTask.cfgSampClkTiming(obj.aoOutputRate, 'DAQmx_Val_FiniteSamps', obj.aoData.samplesPerTrigger);
            obj.hAOTask.cfgOutputBuffer(numel(obj.aoData.volts));
            if ~obj.simulatedDevice
                obj.hAOTask.writeAnalogData(obj.aoData.volts);
            end
            obj.hAOTask.set('startTrigRetriggerable',true);
            obj.hAOTask.control('DAQmx_Val_Task_Verify'); %%% Verify Task Configuration (mostly for trigger routing
        end                
        
        function generateAOData(obj)
            % Updates pre-computed buffer of 'normalized' AO data for FastZ operation
            % 'Normalized' data is properly scaled, but needs to be shifted to the stack starting position
            if ~obj.enable || ~obj.useAOControl
                obj.aoData = struct();
                return
            end
            
            switch obj.waveformType
                case 'sawtooth'
                    % TODO: Is it correct to use the 'absolute coordinates' from the z series here?
                    startVoltage = obj.hDevice.analogCmdPosn2Voltage(obj.hSI.hStackManager.fZs(1));
                    endVoltage   = obj.hDevice.analogCmdPosn2Voltage(obj.hSI.hStackManager.fZs(end));
                    
                    numFramesImaged = obj.hSI.hStackManager.slicesPerAcq;
                    
                    outputRate = obj.aoOutputRate;
                    
                    if obj.discardFlybackFrames && obj.numDiscardFlybackFrames > 0
                        %Flyback/settling will occur during the discarded frame(s) at end of each stack frame set
                        numFramesTotal = numFramesImaged + obj.numDiscardFlybackFrames;
                        obj.fillFraction = numFramesImaged / numFramesTotal;
                        obj.period = (numFramesTotal * obj.hSI.hRoiManager.scanFramePeriod) + obj.volumePeriodAdjustment;
                        
                        %TODO: Deal with negative ramp case
                        %TODO: Detect excessive memory use up front and prevent -- i.e. by warning and disabling FastZ mode
                        
                        totalNumSamples = ceil(obj.period * outputRate);
                        rampNumSamples = round((obj.hSI.hRoiManager.scanFramePeriod * numFramesImaged + obj.volumePeriodAdjustment) * outputRate);
                        assert(rampNumSamples > 0);
                        settlingNumSamples = round(obj.settlingTime * outputRate);
                        
                        flybackNumSamples = totalNumSamples - (rampNumSamples + settlingNumSamples) - 1;
                        assert(flybackNumSamples > 0, 'Error calculating FastZ buffer. volumePeriodAdjustment may be too low/high.');
                        
                        rampData = linspace(startVoltage,endVoltage,rampNumSamples);
                        rampSlope = (endVoltage-startVoltage)/(rampNumSamples-1);
                        
                        settlingStartVoltage = startVoltage - rampSlope * settlingNumSamples;
                        
                        flybackData = linspace(endVoltage,settlingStartVoltage,flybackNumSamples+1);
                        flybackData(1) = [];
                        
                        settlingData = linspace(settlingStartVoltage,startVoltage,settlingNumSamples+1);
                        
                        aoVolts = [rampData flybackData settlingData]';
                    else
                        %Flyback/settling will occur at start of first frame in each stack frame set
                        %Command signal is simply naive...no shaped flyback or settling period
                        numFramesTotal = numFramesImaged;
                        obj.fillFraction = numFramesImaged / numFramesTotal;
                        obj.period = numFramesTotal * obj.hSI.hRoiManager.scanFramePeriod + obj.volumePeriodAdjustment;
                        
                        rampNumSamples = outputRate * obj.period;
                        aoVolts = linspace(startVoltage,endVoltage,rampNumSamples)';
                    end
                    
                    aoDataSlope = (endVoltage-startVoltage)/(rampNumSamples/outputRate);
                    
                    %Shift voltage data to account for acquisition delay
                    shiftVoltage = obj.acquisitionDelay * aoDataSlope;
                    obj.aoData.volts = aoVolts + shiftVoltage;
                    obj.aoData.samplesPerTrigger = numel(aoVolts);
                    obj.aoData.triggerEdge = 'DAQmx_Val_Rising';
            
                case 'step'
                    zs = obj.hSI.hStackManager.fZs;
                    vs = obj.hDevice.analogCmdPosn2Voltage(zs);
                    aoVolts = repmat([vs(2:end) vs(1)],10,1);
                    obj.aoData.volts = reshape(aoVolts,[],1);
                    obj.aoData.samplesPerTrigger = 10;
                    obj.aoData.triggerEdge = 'DAQmx_Val_Falling';

                otherwise
                    error('Unsupported fast Z waveform type.');
            end
        end
    end   
    
    
    %%% ABSTRACT METHOD Implementation (scanimage.interfaces.Component)
    methods (Hidden, Access = protected)
        function componentStart(obj)
            if obj.hSI.hStackManager.isFastZ
                obj.generateAOData();
                
                if strcmp(obj.waveformType, 'step')
                    obj.goTo(obj.hSI.hStackManager.fZs(1));
                end
                
                obj.updateTaskConfiguration();
                
                if ~obj.simulatedDevice
                    obj.hAOTask.start();
                end
            end
        end
        
        function componentAbort(obj)
            if obj.enable && ~isempty(obj.hAOTask)
                obj.hAOTask.abort();
            end
        end
    end
    
    %%%Abstract method impementations (most.Model)
    methods (Access=protected, Hidden)
        function mdlInitialize(obj)
            if isempty(obj.mdfData.fastZControllerType);
                fprintf('No FastZ controller specified in Machine Data File. Feature disabled.\n');
                return
            end
            
            znstInitFastZHardware();
            
            if isempty(obj.hDevice)
                fprintf('Error occurred while initializing FastZ controller. Feature disabled.\n');
                return
            end
            
            obj.numInstances = 1;
            obj.goPark();
            
            mdlInitialize@most.Model(obj);
            
            % Nested functions
            function znstInitFastZHardware()
                useMotor2 = false;
                try
                    %Construct & initialize fastZ object in hardware-specific manner
                    if obj.secondMotor
                        useMotor2 = true;
                        
                        controllerType = obj.hSI.hMotors.mdfData.motor2ControllerType;
                        assert(~isempty(controllerType),'FastZ motor controller was configured as ''useMotor2'', but no secondary Z motor was actually specified.');
                        assert(~isempty(obj.hSI.hMotors.hMotorZ),'FastZ motor controller was configured as ''useMotor2'', but secondary Z motor was not successfully configured.');
                        
                        comPort = obj.hSI.hMotors.mdfData.motor2COMPort;
                        baudRate = obj.hSI.hMotors.mdfData.motor2BaudRate;
                    else
                        controllerType = obj.mdfData.fastZControllerType;
                        comPort = obj.mdfData.fastZCOMPort;
                        baudRate = obj.mdfData.fastZBaudRate;
                    end
                    
                    if useMotor2
                        obj.hDevice = obj.hSI.hMotors.hMotorZ;
                    end
                    
                    %Initialize fastZ AO object, if specified & not done so already
                    if ~isempty(obj.mdfData.fastZAOChanID) && isempty(obj.hAOTask)
                        znstInitFastZAO();
                    end
                    
                    switch lower(controllerType)
                        case {'pi.e517' 'pi.e712'}
                            ctrlType = strrep(controllerType,'pi.',''); % remove leading 'pi.'
                            analogCmdArgs = {};
                            znstInitFastZ(@dabs.pi.LinearStageController,[{'controllerType',ctrlType,'comPort',comPort,'baudRate',baudRate} analogCmdArgs]);
                        
                        case {'pi.e709' 'pi.e753'}
                            ctrlType = 'e753';
                            analogCmdArgs = {};
                            znstInitFastZ(@dabs.pi.LinearStageController7x,[{'controllerType',ctrlType,'comPort',comPort,'baudRate',baudRate} analogCmdArgs]);
                        
                        case {'pi.e816' 'pi.e665'} %E665 uses E816 controller under hood
                            analogCmdArgs = {};
                            znstInitFastZ(@dabs.pi.LinearStageController,[{'controllerType','e816','comPort',comPort,'baudRate',baudRate} analogCmdArgs]);
                            
                        case {'npoint.lc40x'}
                            analogCmdArgs = {};
                            znstInitFastZ(@dabs.npoint.LinearStageController,[{'controllerType','LC40x','comPort',comPort,'baudRate',baudRate} analogCmdArgs]);
                            
                        case {'analog'}
                            if ~useMotor2
                                numDeviceDimensions = numel(obj.mdfData.fastZAOChanID);
                                assert(numDeviceDimensions > 0,'One or more analogCmdChanIDs must be specified for Analog LSC');
                                
                                hLSC = dabs.generic.LSCPureAnalog('numDeviceDimensions',numDeviceDimensions);
                                obj.hDevice = scanimage.components.motors.StageController(hLSC);
                            else
                                hLSC = obj.hSI.hMotors.hMotorZ.hLSC;
                            end
                            
                            %Create/configure fastZAO Task, coordinated with static AO Task maintained by LSC
                            znstInitFastZAO(hLSC);
                            obj.hDevice.initializeAnalogOption('hAOTaskBuffered',obj.hAOTask);

                        case {'simulated.piezo'}
                            analogCmdArgs = {};
                            obj.simulatedDevice = true;
                            znstInitFastZ(@dabs.simulated.Piezo,analogCmdArgs);
                            
                        otherwise
                            assert(false,'FastZ controller type specified (''%s'') is unrecognized or presently unsupported',controllerType);
                    end
                catch ME
                    most.idioms.dispError('Error occurred while initializing fastZ hardware. Incorrect MachineDataFile settings likely cause. \n Disabling motor feature. \n Error stack: \n  %s \n',ME.getReport());
                    
                    if ~isempty(obj.hDevice) && ~useMotor2
                        delete(obj.hDevice);
                    end
                    
                    obj.hDevice = [];
                end
                
                function znstInitFastZ(xtor,xtorArgs)
                    %Require AO Task be used for FastZ, where available
                    znstRequireFastZAO();
                    
                    %TODO: Make ctor call logic programmatic, based on controllerType string
                    if ~useMotor2
                        hLSC = feval(xtor,xtorArgs{:});
                        obj.hDevice = scanimage.components.motors.StageController(hLSC);
                    end
                    
                    %Initialize analog command option
                    args = {'analogCmdBoardID', obj.hAOTask.deviceNames{1},'analogCmdChanIDs',obj.mdfData.fastZAOChanID,'hAOTaskBuffered',obj.hAOTask};
                    if ~isempty(obj.mdfData.fastZAIChanID) && ~isempty(obj.mdfData.fastZDeviceName)
                        args = [args {'analogSensorBoardID' obj.mdfData.fastZDeviceName 'analogSensorChanIDs' obj.mdfData.fastZAIChanID}];
                    end
                    obj.hDevice.initializeAnalogOption(args{:});
                    
                    %Set analog-controllable LSC to use analog mode
                    obj.hDevice.analogCmdEnable = true;
                end
                
                function znstRequireFastZAO()
                    if isempty(obj.mdfData.fastZAOChanID)
                        throwAsCaller(MException('','Analog Output (AO) Task required for specified FastZ hardware type (''%s'')',obj.mdfData.fastZControllerType));
                    end
                    
                    obj.requireAO = true;
                    obj.useAOControl = true;
                end
                
                function znstInitFastZAO(src)
                    
                    if nargin == 0 %Use MDF
                        obj.zprvMDFVerify('fastZDeviceName',{{'char'},{'nonempty'}},[]);
                        obj.zprvMDFVerify('fastZAOChanID',{{'numeric'},{'integer' 'scalar'}},[]);
                        
                        fastZDeviceName = obj.mdfData.fastZDeviceName;
                        fastZAOChanID = obj.mdfData.fastZAOChanID;
                    else
                        fastZDeviceName = src.analogCmdBoardID;
                        fastZAOChanID = src.analogCmdChanIDs(end);
                    end
                    
                    %obj.hAOTask = obj.zprvDaqmxTask('FastZ AO');
                    most.idioms.safeDeleteObj(obj.hAOTask);
                    obj.hAOTask = most.util.safeCreateTask('FastZ AO');
                    obj.hAOTask.createAOVoltageChan(fastZDeviceName,fastZAOChanID);
                    
                    obj.aoOutputRate = obj.hAOTask.get('sampClkMaxRate');
                    
                    obj.hAOTask.cfgSampClkTiming(obj.aoOutputRate, 'DAQmx_Val_FiniteSamps');
                    obj.hAOTask.cfgDigEdgeStartTrig(obj.extFrameClockTerminal);
                    obj.hAOTask.set('startTrigRetriggerable',true);
                    
                    %%% Verify Task Configuration (mostly for trigger routing)
                    obj.hAOTask.control('DAQmx_Val_Task_Verify');
                end
            end
        end
    end
end

%% LOCAL
function s = ziniInitPropAttributes()
    s = struct;
    s.volumesDone = struct('Classes','numeric','Attributes',{{'positive' 'integer' 'finite'}});
    s.enable = struct('Classes','binaryflex','Attributes','scalar');
    s.numVolumes = struct('Classes','numeric','Attributes',{{'positive' 'integer'}});
    s.period = struct('Attributes', 'nonnegative');
    s.numDiscardFlybackFrames = struct('DependsOn',{{'enable' 'numVolumes' 'hSI.scan2DGrabProps' 'hSI.hStackManager.slicesPerAcq' 'acquisitionDelay' 'settlingTime' 'hSI.hRoiManager.scanFrameRate' 'discardFlybackFrames' 'waveformType'}});
    s.useAOControl = struct('Classes','binaryflex','Attributes','scalar');
    s.volumePeriodAdjustment = struct('Range',[-5e-3 5e-3]);
    s.settlingTime = struct('Attributes',{{'nonnegative', '<=', 1}});
    s.acquisitionDelay = struct('Attributes','nonnegative');
    s.userZs = struct('Classes','numeric','Attributes',{{'vector' 'finite'}});
end


%--------------------------------------------------------------------------%
% FastZ.m                                                                  %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
