classdef LSCAnalogOption < dabs.interfaces.LinearStageController
    %LSCANALOGOPTION LinearStageController that provides option for controlling position via an analog signal
    %   Class operates directly as a standard LinearStageController by default
    %   When analogCmdEnable is set True, then analog signal is used for position control
    
    
    %% VISIBLE PROPERTIES
    properties  (Dependent)
        analogCmdEnable; %Logical; if true, analog command signal is in use to control LSC position
    end
        
    %% HIDDEN PROPERTIES
    
    %Self-initialized properties
    properties (Hidden, SetAccess=protected)
        analogCmdBoardID;
        analogCmdChanIDs;
        analogSensorBoardID;
        analogSensorChanIDs;
        
        hAOBuffered; %Handle to an AO NI.DAQmx.Task object used for buffered analog control external to this class

        hAOLSC; %Handle to AO NI.DAQmx.Task used by this class for analog LSC operations
        hAILSC; %Handle to AI NI.DAQmx.Task used by this class for analog input operations
        
        analogCmdOutputRange; %2 element array containining [min max] voltage values allowed for FastZ AO control
        analogPosnOffset;  %Measured offset between command and sensor positions
    end
    
    %% CONSTANT PROPERTIES
    properties (Hidden, Constant)
        CALIBRATION_RANGE_CLAMP_FACTOR = 0.98;  % When performing offset calibration, setpoint has to be within CALIBRATION_RANGE_CLAMP_FACTOR * maximum Voltage range
    end
    
    %% ABSTRACT PROPERTIES
    properties (Abstract,SetAccess=protected,Hidden)        
       analogCmdEnableRaw; %Implements concrete subclass actions, if any, on change of analogCmdEnable
    end
    
    
    %% ABSTRACT PROPERTY REALIZATIONS (dabs.interfaces.LinearStageController)
    
    properties (Dependent,SetAccess=protected)
        isMoving;
    end
    
    properties (Dependent,SetAccess=protected,Hidden)
        positionAbsoluteRaw;
    end
    

    

  %% OBJECT LIFECYCLE    
    methods
        
        function obj = LSCAnalogOption(varargin)       
            % obj = LSCAnalogOption(p1,v1,p2,v2,...)
            %
            % P-V options:
            % analogCmdBoardID: (OPTIONAL) See initializeAnalogOption()
            % analogCmdChanIDs: (OPTIONAL)  See initializeAnalogOption()
            % analogSensorBoardID: (OPTIONAL) See initializeAnalogOption()
            % analogSensorChanIDs: (OPTIONAL)  See initializeAnalogOption()
            % hAOBuffered: (OPTIONAL)  See initializeAnalogOption()
            %            
                        
            % The LinearStageController ignores unrecognized PVs
            obj = obj@dabs.interfaces.LinearStageController(varargin{:});
                        
            pvCell = most.util.filterPVArgs(varargin,{'analogCmdBoardID' 'analogCmdChanIDs' 'hAOBuffered' 'analogSensorBoardID' 'analogSensorChanIDs' });
            pvStruct = struct(pvCell{:});     
            
            %Initialize analog option, if property values are supplied
            analogCmdReqArgs = isfield(pvStruct,{'analogCmdBoardID' 'analogCmdChanIDs'});
            analogSensorReqArgs = isfield(pvStruct,{'analogSensorBoardID' 'analogSensorChanIDs'});
            if any(analogCmdReqArgs) && ~all(analogCmdReqArgs)
                error('The analogCmdBoardID/analogCmdChanIDs properties must either be both provided, or not provided, at construction time.');
            end
            
            if any(analogSensorReqArgs)
                if ~all(analogSensorReqArgs)
                    error('The analogSensorBoardID/analogSensorChanIDs properties must either be both provided, or not provided, at construction time.');
                elseif ~all(analogCmdReqArgs)
                    error('The analogSensor properties can/should only be provided if the analogCmd properties are specified.');
                end
            end
            
            obj.analogPosnOffset = zeros(1,obj.numDeviceDimensions);
                    
            %Initialize analog option directly here if all the required arguments have been provided
            if any(analogCmdReqArgs)
                numActiveDimensions = numel(find(obj.activeDimensions));            
                assert(length(pvStruct.analogCmdChanIDs) == numActiveDimensions,'Number of analog command channels must match the number of active dimensions (%d)',numActiveDimensions);                
                if any(analogSensorReqArgs)
                    assert(length(pvStruct.analogSensorChanIDs) == numActiveDimensions,'Number of analog sensor channels must match the number of active dimensions (%d)',numActiveDimensions);
                end
     
                obj.initializeAnalogOption(varargin{:});
            end
            
            %Property initialization
            if ~isempty(obj.hAOLSC)
                obj.analogCmdEnable = false;
            else
                assert(~obj.analogCmdEnable,'Device of class ''%s'' requires that analogCmdBoardID/analogCmdChanID are specified on construction');
            end
        end

        function delete(obj)
            most.idioms.safeDeleteObj(obj.hAOLSC);
            most.idioms.safeDeleteObj(obj.hAILSC);
        end
    end
    
    %% PROPERTY ACCESS 
    methods
        
        function val = get.analogCmdEnable(obj)
            val = obj.analogCmdEnableRaw;                                    
        end
        
        function set.analogCmdEnable(obj,val)
            validateattributes(val,{'numeric' 'logical'},{'binary' 'scalar'});
            
            if val && isempty(obj.hAOLSC) %#ok<MCSUP>
                assert(~val,'No analog output channel has been configured; cannot set analogCmdEnable=true');
            end        
            
            obj.analogCmdEnableRaw = val; %#ok<MCSUP>
        end
        

        
        function val = get.isMoving(obj)
            
            if obj.analogCmdEnable
                val = obj.isMovingAnalogHook();
            else
                val = obj.isMovingDigitalHook();
            end
            
        end
        
        function posn = get.positionAbsoluteRaw(obj)
            
            if obj.analogCmdEnable && ~isempty(obj.hAILSC)
                posn = obj.analogSensorVoltage2Posn(obj.hAILSC.readAnalogData()); %Convert into positions                
            else
                posn = obj.positionAbsoluteRawDigitalHook();
            end
            
        end
          
        
    end

      
    %% ABSTRACT METHOD IMPLEMENTATIONS (dabs.interfaces.LinearStageController)
    
    methods (Access=protected,Hidden)
        
        function moveStartHook(obj,absTargetPosn)            
           
            if obj.analogCmdEnable
                
                %Unreserve external AO Task that shares LSC command channel
                if ~isempty(obj.hAOBuffered)
                    obj.hAOBuffered.control('DAQmx_Val_Task_Unreserve');
                end
                                
                %Write new AO voltage, ensuring it's within AO range
                absTargetPosn(isnan(absTargetPosn)) = [];
                outVoltage = obj.analogCmdPosn2Voltage(absTargetPosn);
                
                if ~isempty(obj.hAOLSC)
                    outVoltageClamped = max(obj.analogCmdOutputRange(1),min(obj.analogCmdOutputRange(2),outVoltage));
                    
                    if ~isequal(outVoltage,outVoltageClamped)
                        warning('LSCAnalogOption:outOfRange','Linear Stage Controller: Voltage was clamped to output range');
                    end
                    
                    obj.hAOLSC.writeAnalogData(outVoltageClamped);
                end
            else
               obj.moveStartDigitalHook(absTargetPosn); 
            end
            
        end
        
    end 
  
    %% ABSTRACT METHODS
    
    methods (Abstract)
        voltage = analogCmdPosn2Voltage(obj,posn); %Convert LSC position values into analog voltage (scalar function, applies to all dimensions)
        posn = analogSensorVoltage2Posn(obj,voltage); %Convert analog voltage into LSC position values (scalar function, applies to all dimensions) 
    end

    methods (Abstract,Access=protected)        
        posn = positionAbsoluteRawDigitalHook(obj); %Provide default ('digital') readout of LSC's absolute position
        tf = isMovingDigitalHook(obj); %Provide default ('digital') determination of whether LSC is moving when analogCndEnable=false
        moveStartDigitalHook(obj,absTargetPosn); %Provide default ('digital') LSC move behavior when analogCmdEnable=false                        
    end
    
    %'Semi-abstract' methods, with default implementations provided
    methods (Access=protected)
        
        function tf = isMovingAnalogHook(obj) %#ok<MANU>            
                        
            tf = obj.isMovingDigitalHook();                        
            %Subclasses may implement alternative isMoving logic for analog positioning
            
            %             %Verify that final position is within the desired resolution
            %             %before considering the move complete
            %             if ~tf && obj.nonblockingMoveInProgress  && ~isempty(obj.resolution)
            %                 if any(abs(obj.positionAbsolute - obj.lastTargetPosition) > obj.resolution)
            %                     tf = true; %Wait to get closer to desired target position
            %                 end
            %             end
        end                   
        
    end
    
    %% SUPERUSER METHODS
    
    methods (Hidden)
        
        function initializeAnalogOption(obj,varargin)
            %function initializeAnalogOption(obj,p1,v1,p2,v2,...)
            %Initialize the analog command option, configuring an NI DAQmx AO Task to be used for analog control
            %
            % P-V options:
            % analogCmdBoardID: (OPTIONAL) String specifying NI board identifier (e.g. 'Dev1') containing AO channel for LSC control
            % analogCmdChanIDs: (OPTIONAL) Scalar indicating AO channel number (e.g. 0) used for analog LSC control
            % hAOBuffered: (OPTIONAL) Handle to NI.DAQmx AO Task object used by client which also controls same analogCmdBoard/ChannelID for buffered AO operations
            % analogSensorBoardID: (OPTIONAL) String specifying NI board identifier (e.g. 'Dev1') containing AI channel for LSC position sensor
            % analogSensorChanIDs: (OPTIONAL) Scalar indicating AI channel number (e.g. 0) used for analog LSC position sensor
            %
            % offsetNumMeasurements: (Default=20) Number of voltage measurements to take when determining command/sensor offset
            % Notes:
            %   The hAOBuffered option should be provided if the client controls same analogCmdBoard/ChannelID for buffered operations.
            %   This class will ensure those resources are unreserved before issuing LSC commands
                        
            pvCell = most.util.filterPVArgs(varargin,{'analogCmdBoardID' 'analogCmdChanIDs' 'analogSensorBoardID' 'analogSensorChanIDs' 'hAOBuffered' 'skipSensorCalibration' 'maxVoltage' 'minVoltage'});
            pvStruct = most.util.cellPV2structPV(pvCell);
            
%            assert(isempty(obj.hAOLSC),'The ''initalizeAnalogOption'' method can only be called once for objects of class ''%''.',mfilename('class'));
            
            %Handle external buffered-AO Task, if provided
            if isfield(pvStruct,'hAOBuffered')
                hTmp = pvStruct.hAOBuffered;
                assert(isa(hTmp,'dabs.ni.daqmx.Task') && ~isempty(hTmp.channels) && strcmpi(hTmp.taskType,'AnalogOutput'),'Property ''hAOBuffered'' must contain a DAQmx.Task object with one AO channel');            
                obj.hAOBuffered = hTmp;
            end
            
            %Create hAOLSC
            if isfield(pvStruct,'analogCmdBoardID')
                assert(isempty(obj.hAOLSC),'The analog command board & channel have already been initialized');
                assert(isfield(pvStruct,'analogCmdChanIDs'),'Analog Command Chan IDs must be specified in addition to Analog Command Board ID to initialize analog command option');
                obj.hAOLSC = most.util.safeCreateTask(sprintf('Analog LSC %s AO%d', pvStruct.analogCmdBoardID, pvStruct.analogCmdChanIDs));
                obj.hAOLSC.createAOVoltageChan(pvStruct.analogCmdBoardID,pvStruct.analogCmdChanIDs);
                obj.analogCmdBoardID = pvStruct.analogCmdBoardID;
                obj.analogCmdChanIDs = pvStruct.analogCmdChanIDs;
                
                obj.analogCmdOutputRange = [obj.hAOLSC.channels(1).get('min') obj.hAOLSC.channels(1).get('max')]; %Determine/cache AO range
                if isfield(pvStruct,'maxVoltage')
                    obj.analogCmdOutputRange = min(obj.analogCmdOutputRange, pvStruct.maxVoltage);
                end
                if isfield(pvStruct,'minVoltage')
                    obj.analogCmdOutputRange = max(obj.analogCmdOutputRange, pvStruct.minVoltage);
                end
            end
            
            %Create hAILSC, if specified
            if isfield(pvStruct,'analogSensorBoardID')
                assert(isempty(obj.hAILSC),'The analog sensor board & channel have already been initialized');
                assert(isfield(pvStruct,'analogSensorChanIDs'),'Analog Sensor Chan IDs must be specified in addition to Analog Sensor Board ID to initialize analog sensor option');

                obj.hAILSC = most.util.safeCreateTask(sprintf('Analog LSC %s AI%d', pvStruct.analogSensorBoardID, pvStruct.analogSensorChanIDs));
                obj.hAILSC.createAIVoltageChan(pvStruct.analogSensorBoardID,pvStruct.analogSensorChanIDs);
                obj.analogSensorBoardID = pvStruct.analogSensorBoardID;
                obj.analogSensorChanIDs = pvStruct.analogSensorChanIDs;
            end
			
			if isfield(pvStruct,'skipSensorCalibration')
				skipSensorCalibration = pvStruct.skipSensorCalibration;
			else
				skipSensorCalibration = false;
			end
                                                
            %Determine analogPosnOffset. Offset encompasses any offset on
            %the AO & AI channels, as well as those between the LSC command
            %and monitor signals themselves 
            %
            %NOTE - this does not address any offset variation with position,
            %i.e. due to nonlinearities
            
            if ~isempty(obj.hAILSC) && ~skipSensorCalibration
                assert(~isempty(obj.hAOLSC));
                
                cachedAnalogCmdEnable = obj.analogCmdEnable;
                obj.analogCmdEnable = true;
                
                if isfield(pvStruct,'offsetNumMeasurements')
                    numReadings = pvStruct.offsetNumMeasurements;
                else
                    numReadings = 5000;
                end
                
                initialCmdVoltage = obj.analogCmdPosn2Voltage(0); 
                
                % clamp the sensor voltage to the maximum range
                measRange = obj.analogCmdOutputRange * obj.CALIBRATION_RANGE_CLAMP_FACTOR;
                initialCmdVoltage = max(measRange(1),min(measRange(2),initialCmdVoltage));
                
                obj.hAOLSC.writeAnalogData(initialCmdVoltage);
                pause(0.1);
                testSensorVoltage = mean(obj.hAILSC.readAnalogData(numReadings));
                                
                obj.analogPosnOffset = obj.analogSensorVoltage2Posn(testSensorVoltage);
                
                obj.analogCmdEnable = cachedAnalogCmdEnable;
            end
        end
    end
end



%--------------------------------------------------------------------------%
% LSCAnalogOption.m                                                        %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
