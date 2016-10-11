classdef LSCPureAnalog < dabs.interfaces.LSCAnalogOption & most.HasMachineDataFile
    %LSCPureAnalog Summary of this class goes here
    %   Detailed explanation goes here

    %% PROPERTIES (Constructor-initialized)
    properties (SetAccess=immutable)
        
        commandVoltsPerMicron; %Conversion factor for command signal to analog linear stage controller
        commandVoltsOffset; %Offset value, in volts, for command signal to analog linear stage controller
        
        sensorVoltsPerMicron; %Conversion signal for sensor signal from analog linear stage controller
        sensorVoltsOffset; %Offset value, in volts, for sensor signal from analog linear stage controller
                
    end
    
    %% HIDDEN PROPS
    properties (Hidden, SetAccess=private)
       analogOptionInitialized = false;        
    end
    
    %% ABSTRACT PROPERTY REALIZATIONS (most.HasMachineDataFile)
    properties (Constant, Hidden)
        %Value-Required properties
        mdfClassName = mfilename('class');
        mdfHeading = 'LSC Pure Analog';
        
        %Value-Optional properties
        mdfDependsOnClasses;
        mdfDirectProp; %#ok<MCCPI>
        mdfPropPrefix; %#ok<MCCPI>
    end
    
    %% ABSTRACT PROPERTY REALIZATION (dabs.interfaces.LSCAnalogOption)
    properties (SetAccess=protected,Hidden)
        analogCmdEnableRaw = false
    end
        
        
    %% ABSTRACT PROPERTY REALIZATION (dabs.interface.LinearStageController)
    
    properties (Constant, Hidden)
        nonblockingMoveCompletedDetectionStrategy = 'poll'; % Either 'callback' or 'poll'. If 'callback', this class guarantees that moveDone() will be called when a nonblocking move is complete. See documentation for moveStartHook().
    end
    
    properties (SetAccess=protected,Dependent)
        infoHardware;
    end
    
    properties (SetAccess=protected,Dependent,Hidden)
        velocityRaw;
        accelerationRaw;

        invertCoordinatesRaw;
        maxVelocityRaw;

    end
    
    properties (SetAccess=protected, Hidden)
        resolutionRaw;
        
        positionDeviceUnits = 1e-6;
        velocityDeviceUnits = 1e-6;
        accelerationDeviceUnits = 1e-6;
    end   
    
    
    
    %% OBJECT LIFECYCLE
    methods
        function obj = LSCPureAnalog(varargin)
            
            %REMOVE
            % obj = LSCPureAnalog(p1,v1,p2,v2,...)
            %
            % P-V options:
            % hAOBuffered: (OPTIONAL)  Handle to NI.DAQmx AO Task object used by client which also controls same analogCmdBoard/ChannelID for buffered AO operations 
            %
                        
            
            %Construct dabs.interfaces.analogLSC
            obj = obj@dabs.interfaces.LSCAnalogOption(varargin{:},'skipSensorCalibration',true);
            
            %Initialize transform (scale/offset) properties
            xformProps = {'commandVoltsPerMicron' 'sensorVoltsPerMicron' 'commandVoltsOffset' 'sensorVoltsOffset'}; 
                  
            for i=1:length(xformProps)
                obj.(xformProps{i}) = obj.mdfData.(xformProps{i});
            end
            
            %Initiallize dabs.interfaces.analogLSC
            pvCell = most.util.filterPVArgs(varargin,{'hAOBuffered'},{});
            pvStruct = most.util.cellPV2structPV(pvCell);            
            
            argList = { ...
                'analogCmdBoardID', obj.mdfData.analogCmdBoardID, ...
                'analogCmdChanIDs', obj.mdfData.analogCmdChanIDs, ...
                'analogSensorBoardID', obj.mdfData.analogSensorBoardID, ...
                'analogSensorChanIDs', obj.mdfData.analogSensorChanIDs, ...
                'skipSensorCalibration',true};
            
            if isfield(pvStruct,'hAOBuffered')
                argList = [argList {'hAOBuffered' pvStruct.hAOBuffered}];
            end
            
            %Apply limits if supplied
            minV = -inf;
            maxV = inf;
            if isfield(obj.mdfData,'maxCommandVolts') && ~isempty(obj.mdfData.maxCommandVolts)
                maxV = obj.mdfData.maxCommandVolts;
            end
            if isfield(obj.mdfData,'maxCommandPosn') && ~isempty(obj.mdfData.maxCommandPosn)
                maxV = min(maxV,obj.analogCmdPosn2Voltage(obj.mdfData.maxCommandPosn));
            end
            if isfield(obj.mdfData,'minCommandVolts') && ~isempty(obj.mdfData.minCommandVolts)
                minV = obj.mdfData.minCommandVolts;
            end
            if isfield(obj.mdfData,'minCommandPosn') && ~isempty(obj.mdfData.minCommandPosn)
                minV = max(minV,obj.analogCmdPosn2Voltage(obj.mdfData.minCommandPosn));
            end
            if ~isinf(minV)
                argList = [argList {'minVoltage' minV}];
            end
            if ~isinf(maxV)
                argList = [argList {'maxVoltage' maxV}];
            end
            
            %Initialize analog option for parent LSCAnalogOption class
            obj.initializeAnalogOption(argList{:});
            obj.analogOptionInitialized = true;
            
            %Turn on analog 'option' (which must be/remain True, for this 'pure' analog class)
            obj.analogCmdEnable = true; 
            
            % Calibrate
			[obj.sensorVoltsPerMicron,obj.sensorVoltsOffset] = obj.calibrateSensor();
        end
    end
    
    %% PROPERTY ACCESS METHODS
    methods
        
        function set.analogCmdEnableRaw(obj,val)
            %Coerce True value for analogCmdEnable
            if val ~= true                
                if obj.analogOptionInitialized %Don't warn during initializeAnalogOption() call
                    warning('Cannot set analogCmdEnable to value False for objects of class ''%s''',mfilename('class'));
                end
            end
            obj.analogCmdEnableRaw = true;
        end            
        
        function val = get.infoHardware(~)
            val = 'Analog Only FastZ Actuator Device';                                    
        end
        
        function val = get.invertCoordinatesRaw(obj)
            val = false(1,obj.numDeviceDimensions);
        end        
                
        function val = get.velocityRaw(~)
            val = nan;
        end
        
        function val = get.accelerationRaw(~)
            val = nan;
        end
        
        function val = get.maxVelocityRaw(~)
            val = nan;
        end  
   
    end
    
    %% ABSTRACT METHOD IMPLEMENTATION  (dabs.interfaces.LSCAnalogOption)
    
    methods
        function voltage = analogCmdPosn2Voltage(obj,posn)
            %Convert LSC position values into analog voltage (scalar function, applies to all dimensions)
            voltage = obj.commandVoltsPerMicron * posn + obj.commandVoltsOffset;
        end
        
        function posn = analogSensorVoltage2Posn(obj,voltage)
            %Convert analog voltage into LSC position values (scalar function, applies to all dimensions)
            posn = (voltage - obj.sensorVoltsOffset)  / obj.sensorVoltsPerMicron;
        end
    end
    
    methods (Access = protected, Hidden)
        function posn = voltage2AnalogCmdPosn(obj,voltage)
            %not an abstract implementation, just implemented for convenience
            %Convert analog voltage (scalar function, applies to all dimensions) into LSC position values
            posn = (voltage - obj.commandVoltsOffset) / obj.commandVoltsPerMicron;
        end
    end
    
    methods (Access=protected, Hidden)
        function posn = positionAbsoluteRawDigitalHook(obj)
            %Provide default ('digital') readout of LSC's absolute position
            error('Objects of class ''%s'' do not support digital position readout',mfilename('class'));
        end
        
        function tf = isMovingDigitalHook(obj)
            %Provide default ('digital') determination of whether LSC is moving when analogCndEnable=false
            error('Objects of class ''%s'' do not support digital readout of isMoving status',mfilename('class'));
        end
        
        function moveStartDigitalHook(obj,absTargetPosn)
            %Provide default ('digital') LSC move behavior when analogCmdEnable=false
            error('Objects of class ''%s'' do not support digital move operations',mfilename('class'));
        end
        
        function recoverHook(~)
            %nothing to do to recover
        end
        
        function [sensorVoltsPerMicron_,sensorVoltsOffset_] = calibrateSensor(obj)
            if isempty(obj.sensorVoltsPerMicron) || isempty(obj.sensorVoltsOffset)
                assert(~isempty(obj.hAOLSC) && ~isempty(obj.hAILSC));
                
                numReadings = 5000;
                
                pt1 = obj.voltage2AnalogCmdPosn(0);
                pt2 = obj.voltage2AnalogCmdPosn(obj.analogCmdOutputRange(2) * 0.5);
                
                pt1V = obj.analogCmdPosn2Voltage(pt1);
                pt2V = obj.analogCmdPosn2Voltage(pt2);
                
                % clamp the sensor voltage to the maximum range
                clampedRange = obj.analogCmdOutputRange * obj.CALIBRATION_RANGE_CLAMP_FACTOR;
                assert(pt1V >= clampedRange(1) && pt1V <= clampedRange(2));
                assert(pt2V >= clampedRange(1) && pt2V <= clampedRange(2));
                
                fprintf('Calibrating feedback sensor for pure analog linear stage controller...\n');
                
                obj.hAOLSC.writeAnalogData(pt1V);
                pause(0.5); % wait for stage to settle
                pt1VSens = mean(obj.hAILSC.readAnalogData(numReadings));
                
                obj.hAOLSC.writeAnalogData(pt2V);
                pause(0.5); % wait for stage to settle
                pt2VSens = mean(obj.hAILSC.readAnalogData(numReadings));
                
                sensorVoltsPerMicron_ = (pt2VSens-pt1VSens)/(pt2-pt1);
                sensorVoltsOffset_ = pt1VSens - sensorVoltsPerMicron_ * pt1;
                
                fprintf('\b Done!\n');
            else
                sensorVoltsPerMicron_ = obj.sensorVoltsPerMicron;
                sensorVoltsOffset_ = obj.sensorVoltsOffset;
            end
        end
    end
        
    %Method overrides
    methods (Access=protected)
         function tf = isMovingAnalogHook(obj)
             numReadings = 10; 
             
             initialPosition = mean(obj.hAILSC.readAnalogData(numReadings));
             pause(0.1);
             finalPosition = mean(obj.hAILSC.readAnalogData(numReadings));
             
             tf = any(abs(finalPosition-initialPosition) > obj.resolutionBest);
         end                
    end
    
end



%--------------------------------------------------------------------------%
% LSCPureAnalog.m                                                          %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
