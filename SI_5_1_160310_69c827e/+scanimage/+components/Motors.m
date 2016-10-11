classdef Motors < scanimage.interfaces.Component & most.HasMachineDataFile
    %% USER PROPS
    properties (SetObservable)
        userDefinedPositions = repmat(struct('name','','coords',[]),0,1);
    end
    
    properties (SetObservable, GetObservable, Transient)
        motorSecondMotorZEnable = false;        %scalar logical. If true, use second motor for stack z-movement. This flag is only interesting when motorDimensionConfiguration is 'xyz-z'. (For other motorDimensionConfigurations, the value of this flag is constrained to a single value.)
        motorDimensionConfiguration;            %one of {'none' 'xy' 'z' 'xyz'} when there is a single motor; one of {'xy-z' 'xyz-z'} when there are two motors
        motorMoveTimeout = 5;                   %Maximum time, in seconds, to allow for stage moves. %TODO: Ideally could anticipate
        motorFastMotionThreshold = 100;         %Distance, in um, above which motion will use the 'fast' velocity for controller
        motorPosition;                          %1x3 or 1x4 array specifying motor position (in microns), depending on single vs dual motor, and motorDimensionConfiguration.
    end
    
    %% FRIEND PROPS
    properties (Hidden,SetObservable,SetAccess=?scanimage.interfaces.Class)
        hMotor;                                 %Warning: It is dangerous to directly zero or modify the relative coordinate system on the motor. This will break stackZStart/EndPos. See motorZeroSoft().
        hMotorZ;
        stackZMotor;                            %handle to motor user for stack z-positioning during acq
        hErrorCallBack;                         %Function handle for Motor Error (should be set by SI.m)
        
        stackCurrentMotorZPos;                  %z-position of stackZMotor
        stackHomeZPos                           %cached home position to return to at end of stack
    end
    
    %% INTERNAL PROPS
    properties (Hidden, Transient)
        motorPositionLength;                    %Length of motorPosition values
    end
    
    %%% ABSTRACT PROPERTY REALIZATIONS (most.HasMachineDataFile)
    properties (Constant, Hidden)
        %Value-Required properties
        mdfClassName = mfilename('class');
        mdfHeading = 'Motors';
        
        %Value-Optional properties
        mdfDependsOnClasses; %#ok<MCCPI>
        mdfDirectProp;       %#ok<MCCPI>
        mdfPropPrefix;       %#ok<MCCPI>
    end
    
    %%% ABSTRACT PROPERTY REALIZATION (most.Model)
    properties (Hidden, SetAccess=protected)
        mdlPropAttributes = ziniInitPropAttributes();
        mdlHeaderExcludeProps = {'hMotor','hMotorZ'};                            
    end
    
    %%% ABSTRACT PROPERTY REALIZATION (scanimage.interfaces.Component)
    properties (SetAccess = protected, Hidden)
        numInstances = 0;
    end
    
    properties (Constant, Hidden)
        COMPONENT_NAME = 'Motors';                                         % [char array] short name describing functionality of component e.g. 'Beams' or 'FastZ'
        PROP_TRUE_LIVE_UPDATE = {'motorPosition','stackCurrentMotorZPos'}; % Cell array of strings specifying properties that can be set while the component is active
        PROP_FOCUS_TRUE_LIVE_UPDATE = {};                                  % Cell array of strings specifying properties that can be set while focusing
        DENY_PROP_LIVE_UPDATE = {'motorMoveTimeout',...                    % Cell array of strings specifying properties for which a live update is denied (during acqState = Focus)
            'motorSecondMotorZEnable','motorFastMotionThreshold'};
        
        FUNC_TRUE_LIVE_EXECUTION = {'motorZeroSoft','zprvGoHome',...       % Cell array of strings specifying functions that can be executed while the component is active
            'zprvResetHome'};
        FUNC_FOCUS_TRUE_LIVE_EXECUTION = {'motorZeroXYZ','motorZeroXY','motorZeroZ'};         % Cell array of strings specifying functions that can be executed while focusing
        DENY_FUNC_LIVE_EXECUTION = {'saveUserDefinedPositions' 'loadUserDefinedPositions'};    % Cell array of strings specifying functions for which a live execution is denied (during acqState = Focus)
    end
    
    %% LIFECYCLE
    methods (Hidden)
        function obj = Motors(hSI)
            obj = obj@scanimage.interfaces.Component(hSI);
            obj.hMotor = [];
            obj.hMotorZ = [];
            
            if isempty(obj.mdfData.motorControllerType)
                if ~isempty(obj.mdfData.motor2ControllerType)
                    error('SI:motorInitErr',...
                        'A secondary z-dimension motor controller was specified without specifying a primary motor controller. This is not supported.');
                end
                fprintf(1,'No motor controller specified in Machine Data File. Feature disabled.\n');
                return;
            end
            
            try
                [obj.hMotor, mtrDims1] = obj.ziniMotorConfigureAndConstruct('motor',false);
            catch ME
                most.idioms.dispError('Error constructing/initializing primary motor:\n%s\n',ME.message);
                most.idioms.dispError('Disabling motor feature.\n');
            end
            
            if isempty(obj.hMotor)
                return
            else
                obj.numInstances = 1;
            end
            
            if ~isempty(obj.mdfData.motor2ControllerType)
                try
                    [obj.hMotorZ, mtrDims2] = obj.ziniMotorConfigureAndConstruct('motor2',true);
                catch ME
                    most.idioms.dispError('Error constructing/initializing secondary motor:\n%s\n',ME.message);
                    most.idioms.dispError('Disabling secondary motor.\n');
                end
            end
            
            if ~isempty(obj.hMotorZ)
               obj.numInstances = obj.numInstances + 1; 
            end
            
            if isempty(obj.hMotorZ)
                obj.motorDimensionConfiguration = mtrDims1;
            else
                obj.motorDimensionConfiguration = sprintf('%s-%s',mtrDims1,mtrDims2);
            end
            
            switch obj.motorDimensionConfiguration
                case {'xyz' 'xy' 'z' 'xyz-z'}
                    obj.motorSecondMotorZEnable = false;
                case {'xy-z'}
                    obj.motorSecondMotorZEnable = true;
            end
            
            % The following callbacks have to be added to the SI object,
            % otherwise acquisition cannot be cancelled in case of a motor
            % failure.
            if obj.numInstances >= 1
                obj.hMotor.addlistener('LSCError',@obj.hErrorCallBack);
            end
            if obj.numInstances >= 2
                obj.hMotorZ.addlistener('LSCError',@obj.hErrorCallBack);
            end
        end
    end
    
    methods (Access = protected, Hidden)
        function mdlInitialize(obj)
            if obj.numInstances > 0
                mdlInitialize@most.Model(obj);
            end
        end
    end
    
    %% PROP ACCESS
    methods        
        % MOTOR SPECIFIC PROPERTY ACCESS METHODS
        function val = get.motorDimensionConfiguration(obj)
            if obj.numInstances >= 1
                val = obj.motorDimensionConfiguration;
            else
                %Just use the error value here.
                val = 'error';
            end
        end
        
        function set.motorDimensionConfiguration(obj,val)
            obj.motorDimensionConfiguration = val;
        end
        
        function val = get.motorPosition(obj)
            if obj.numInstances <= 0
                val = [];
            else
                val = obj.hMotor.positionRelative;
                if obj.numInstances >= 2
                    secZPos = obj.hMotorZ.positionTarget(3);
                    switch obj.motorDimensionConfiguration
                        case 'xy-z'
                            val(3) = secZPos;
                        case 'xyz-z'
                            val(4) = secZPos;
                        otherwise
                            assert(false,'Impossible value of motorDimensionconfiguration');
                    end
                end
            end
        end
        
        function val = get.motorPositionLength(obj)
            if obj.numInstances <= 0
                val = 0;
            elseif obj.numInstances == 1 || strcmpi(obj.motorDimensionConfiguration,'xy-z')
                val = 3;
            else
                val = 4;
            end
        end
        
        function set.motorMoveTimeout(obj,val)
            val = obj.validatePropArg('motorMoveTimeout',val);
            if obj.componentUpdateProperty('motorMoveTimeout',val)
                
                %Currently a single SI moveTimeout property controls the
                %primary and secondary motor move and async-move timeout values
                obj.hMotor.nonblockingMoveTimeout = val; 
                obj.hMotor.moveTimeout = val;
                if obj.numInstances >= 2 
                    obj.hMotorZ.nonblockingMoveTimeout = val; 
                    obj.hMotorZ.moveTimeout = val; 
                end
                obj.motorMoveTimeout = val;
            end
        end
        
        function set.motorFastMotionThreshold(obj,val)
            val = obj.validatePropArg('motorFastMotionThreshold',val);
            if obj.componentUpdateProperty('motorFastMotionThreshold',val)
                obj.hMotor.twoStepDistanceThreshold = val; 
                obj.motorFastMotionThreshold = val;
            end
        end
        
        function set.motorSecondMotorZEnable(obj,val)            
            obj.validatePropArg('motorSecondMotorZEnable',val);
            if obj.componentUpdateProperty('motorSecondMotorZEnable',val)
                mdc = obj.motorDimensionConfiguration; 
                switch mdc
                    case {'xyz' 'xy' 'z'}
                        assert(~logical(val),...
                            'Cannot enable second motor when motorDimensionConfiguration is ''%s''.',mdc);
                    case 'xy-z'
                        assert(logical(val),...
                            'Second motor must be enabled when motorDimensionConfiguration is ''%s''.',mdc);
                    case 'xyz-z'
                        %none
                end
                
                obj.motorSecondMotorZEnable = val;
                
                obj.hSI.hStackManager.stackClearStartEnd();
            end
        end

        function val = get.motorSecondMotorZEnable(obj)
            if obj.numInstances >= 2
                val = obj.motorSecondMotorZEnable;
            else
                val = false;
            end
        end
                
        function set.motorPosition(obj,val)
            val = obj.validatePropArg('motorPosition',val);
            if obj.componentUpdateProperty('motorPosition',val)
                val = val(:)';
                
                if obj.numInstances >= 2 
                    switch obj.motorDimensionConfiguration 
                        case 'xy-z'
                            assert(numel(val)==3);
                            
                            currentPos = obj.hMotor.positionRelative(:)'; 
                            if ~isequal(val(1:2),currentPos(1:2))
                                obj.hMotor.moveCompleteRelative([val(1:2) nan]);
                            end
                            
                            if ~isequal(val(3),obj.hMotorZ.positionTarget(3))
                                obj.hMotorZ.moveCompleteRelative([ nan nan val(3)]); 
                            end
                        case 'xyz-z'
                            assert(numel(val)==4);
                            
                            if ~isequal(val(1:3),obj.hMotor.positionRelative(:)') 
                                obj.hMotor.moveCompleteRelative(val(1:3)); %#ok<*MCSUP>
                            end
                            if ~isequal(val(4),obj.hMotorZ.positionTarget(3))
                                obj.hMotorZ.moveCompleteRelative([ nan nan val(4)]);
                            end
                        otherwise
                            assert(false);
                    end
                    %TODO (??): Maybe implement FastZPosnGotoAO() operation
                    %here..i.e. go to position using either digital
                    %(moveComplete) or analog (FastZPosnGotoAO) operation
                else
                    assert(numel(val)==3,'Motor position should have three elements.')
                    if ~isequal(val,obj.hMotor.positionRelative) % clause is redundant
                        obj.hMotor.moveCompleteRelative(val);
                    end
                end
            end
        end
        
        function set.userDefinedPositions(obj,val)
            assert(all(isfield(val,{'name' 'coords'})), 'Invalid setting for userDefinedPositions');
            obj.userDefinedPositions = val;
        end
        
        function val = get.stackZMotor(obj)
            if obj.numInstances <= 0
                val = [];
                return;
            end
            
            if obj.motorSecondMotorZEnable
                assert(obj.numInstances >= 2);
                val = obj.hMotorZ;
            else
                val = obj.hMotor;
            end
        end
        
        function val = get.stackCurrentMotorZPos(obj)
            if obj.stackZMotor.stackStartReadPos
                val = obj.stackZMotor.positionRelative(3);
            else
                val = obj.stackZMotor.positionTarget(3);
            end
        end
        
        function set.stackCurrentMotorZPos(obj,val)
%             if obj.componentUpdateProperty('stackCurrentMotorZPos',val)
                %always allow this. needed for stack operation
                obj.stackZMotor.moveCompleteRelative([nan nan val]);
%             end
        end
    end    
    
    %% USER METHODS
    methods
        function zprvResetHome(obj)
            if obj.componentExecuteFunction('zprvResetHome')
                obj.stackHomeZPos = [];
            end
        end
        function zprvSetHome(obj)
            if obj.componentExecuteFunction('zprvResetHome')
                obj.stackHomeZPos = obj.stackCurrentMotorZPos;
            end
        end
        
        function zprvGoHome(obj)
            if obj.componentExecuteFunction('zprvGoHome')
                if ~isempty(obj.stackHomeZPos)
                    obj.stackCurrentMotorZPos = obj.stackHomeZPos;
                end
            end
        end
        
        function zprvGoPark(obj)
            if obj.componentExecuteFunction('zprvGoPark')
                % Do nothing for motors.
            end
        end

        function motorZeroXYZ(obj)
            %Set motor relative origin to current position for X,Y,and Z coordinates.
            if obj.componentExecuteFunction('motorZeroXYZ')
                switch obj.motorDimensionConfiguration
                    case 'xy'
                        obj.motorZeroSoft(logical([1 1 0]));
                    case 'z'
                        obj.motorZeroSoft(logical([0 0 1]));
                    case 'xyz'
                        obj.motorZeroSoft(logical([1 1 1]));
                    case 'xy-z'
                        obj.motorZeroSoft(logical([1 1 1]));
                    case 'xyz-z'
                        obj.motorZeroSoft(logical([1 1 1 0])); %Do not zero secondary-Z; require motorZeroZ() to do this, with motorSecondMotorZEnable=true
                end
            end
        end
        
        function motorZeroXY(obj)
            %Set motor relative origin to current position for X&Y coordinates.
            if obj.componentExecuteFunction('motorZeroXY')
                switch obj.motorDimensionConfiguration
                    case 'xy'
                        obj.motorZeroSoft(logical([1 1 0]));
                    case 'z'
                        % none
                    case 'xyz'
                        obj.motorZeroSoft(logical([1 1 0]));
                    case 'xy-z'
                        obj.motorZeroSoft(logical([1 1 0]));
                    case 'xyz-z'
                        obj.motorZeroSoft(logical([1 1 0 0]));
                end
            end
        end
        
        function motorZeroZ(obj)
            %Set motor relative origin to current position for Z
            %coordinates. Honor motorSecondMotorZEnable property, if
            %applicable.
            if obj.componentExecuteFunction('motorZeroZ')
                switch obj.motorDimensionConfiguration
                    case 'xy'
                        % none
                    case 'z'
                        obj.motorZeroSoft(logical([0 0 1]));
                    case 'xyz'
                        obj.motorZeroSoft(logical([0 0 1]));
                    case 'xy-z'
                        obj.motorZeroSoft(logical([0 0 1]));
                    case 'xyz-z'
                        if obj.motorSecondMotorZEnable
                            obj.motorZeroSoft(logical([0 0 0 1]));
                        else
                            obj.motorZeroSoft(logical([0 0 1 0]));
                        end
                        
                end
            end
        end
        
        function defineUserPosition(obj,name,posn)
            % Add current motor position, or specified posn, to
            % motorUserDefinedPositions array at specified idx
            if nargin < 2 || isempty(name)
                name = '';
            end
            if nargin < 3 || isempty(posn)
                posn = obj.motorPosition;
            end
            obj.userDefinedPositions(end+1) = struct('name',name,'coords',posn);
        end
        
        % Clears all user-defined positions
        function clearUserDefinedPositions(obj)
            obj.userDefinedPositions = repmat(struct('name','','coords',[]),0,1);
        end
        
        function gotoUserDefinedPosition(obj,posn)
            %Move motor to stored position coordinates
            if obj.componentExecuteFunction('gotoUserDefinedPosition',posn)
                if ischar(posn)
                    posn = ismember(posn, {obj.userDefinedPositions.name});
                end
                assert(posn > 0 && numel(obj.userDefinedPositions) >= posn, 'Invalid position selection.');
                obj.motorPosition = obj.userDefinedPositions(posn).coords;
            end
        end
        
        function saveUserDefinedPositions(obj)
            %Save contents of motorUserDefinedPositions array to a position (.POS) file
            if obj.componentExecuteFunction('motorSaveUserDefinedPositions')
                [fname, pname]=uiputfile('*.pos', 'Choose position list file'); % TODO starting path
                if ~isnumeric(fname)
                    periods=strfind(fname, '.');
                    if any(periods)
                        fname=fname(1:periods(1)-1);
                    end
                    s.motorUserDefinedPositions = obj.motorUserDefinedPositions; %#ok<STRNU>
                    save(fullfile(pname, [fname '.pos']),'-struct','s','-mat');
                end
            end
        end
        
        function loadUserDefinedPositions(obj)
            %Load contents of a position (.POS) file to the motorUserDefinedPositions array (overwriting any previous contents)
            if obj.componentExecuteFunction('motorLoadUserDefinedPositions')
                [fname, pname]=uigetfile('*.pos', 'Choose position list file');
                if ~isnumeric(fname)
                    periods=strfind(fname,'.');
                    if any(periods)
                        fname=fname(1:periods(1)-1);
                    end
                    s = load(fullfile(pname, [fname '.pos']), '-mat');
                    obj.motorUserDefinedPositions = s.motorUserDefinedPositions;
                end
            end
        end
    end   
    
    %% INTERNAL METHODS
    methods (Access = private, Hidden)
         function motorZeroSoft(obj,coordFlags)
            % Do a soft zero along the specified coordinates, and update
            % stackZStart/EndPos appropriately.
            %
            % SYNTAX
            % coordFlags: a 3- or 4-element logical vec. The number of
            % elements should match motorPositionLength.
            %
            % NOTE: it is a bit dangerous to expose the motor publicly, since
            % zeroing it directly will bypass updating stackZStart/EndPos.
            if obj.componentExecuteFunction('motorZeroSoft')
                coordFlags = logical(coordFlags);
                assert(numel(coordFlags)==obj.motorPositionLength,...
                    'Number of elements in coordFlags must match motorPositionLength.');
                
                if strcmp(obj.motorDimensionConfiguration,'xyz-z') && obj.motorSecondMotorZEnable
                    tfRescaleStackZStartEndPos = coordFlags(4);
                else
                    tfRescaleStackZStartEndPos = coordFlags(3);
                end
                if tfRescaleStackZStartEndPos
                    origZCoord = obj.stackZMotor.positionTarget(3);
                end
                
                switch obj.motorDimensionConfiguration
                    case {'xyz' 'xy' 'z'}
                        obj.hMotor.zeroSoft(coordFlags);
                    case 'xy-z'
                        obj.hMotor.zeroSoft([coordFlags(1:2) false]);
                        obj.hMotorZ.zeroSoft([false false coordFlags(3)]);
                    case 'xyz-z'
                        obj.hMotor.zeroSoft(coordFlags(1:3));
                        if numel(coordFlags)==4
                            obj.hMotorZ.zeroSoft([false false coordFlags(4)]);
                        end
                end
                
                if tfRescaleStackZStartEndPos
                    obj.hSI.hStackManager.stackZStartPos = obj.hSI.hStackManager.stackZStartPos-origZCoord;
                    obj.hSI.hStackManager.stackZEndPos = obj.hSI.hStackManager.stackZEndPos-origZCoord;
                end
            end            
         end
        
        function [motorObj, mtrDims] = ziniMotorConfigureAndConstruct(obj,mdfPrefix,tfIsSecondaryMotor)
            
            % Get controller type
            type = obj.mdfData.(sprintf('%s%s',mdfPrefix,'ControllerType'));
            
            if type(1) == '+'
                % Optional component is specified as motor controller
                type(1) = 'h';
                
                % First make sure it loaded
                assert(isprop(obj.hSI, type), 'Optional component ''%s'' was specified as the motor controller but the component did not load successfully.', type(2:end));
                
                % Next make sure it supports proper interfaces
                assert(isa(obj.hSI.(type),'dabs.interfaces.LinearStageController'), 'Optional component ''%s'' was specified as the motor controller but it does not support the ''dabs.interfaces.LinearStageController'' interface.', type(2:end));
                assert(isa(obj.hSI.(type),'scanimage.interfaces.LscComponent'), 'Optional component ''%s'' was specified as the motor controller but it does not support the ''scanimage.interfaces.LscComponent'' interface.', type(2:end));
                
                % Finally make sure it successfully initialized
                assert(obj.hSI.(type).lscInitSuccessful, 'Optional component ''%s'' was specified as the motor controller but it reports that the stage failed to initialize.', type(2:end));
                
                lscObj = obj.hSI.(type);
                mtrDims = lscObj.motorDimensions;
                regInfo = lscObj.motorRegistryInfo;
            else
                % Get controller info
                regInfo = scanimage.components.motors.MotorRegistry.getControllerInfo(lower(type));
                
                assert(~isempty(regInfo), ['Specified motor type (''' type ''') not found in motor registry.']);
                
                % Construct/init LSC
                [lscObj, mtrDims] = obj.ziniMotorLSCConstruct(regInfo,mdfPrefix,tfIsSecondaryMotor);
            end
            
            
            % Construct StageController
            motorObj = obj.ziniMotorStageControllerConstruct(regInfo,lscObj,mdfPrefix);
        end
        
        function [lsc, mtrDims] = ziniMotorLSCConstruct(obj,info,mdfPrefix,tfIsSecondaryMotor)
            
            % Compile arguments for LSC construction
            lscArgs = struct();
            
            if ~isempty(info.SubType)
                lscArgs.controllerType = info.SubType;
            end
            
            stageType = obj.mdfData.(sprintf('%s%s',mdfPrefix,'StageType'));
            lscArgs.stageType = stageType;
            
            optionalArgMap = containers.Map({'PositionDeviceUnits' 'COMPort' 'BaudRate'},...
                {'positionDeviceUnits' 'comPort' 'baudRate'});
            for key = optionalArgMap.keys
                mdfOptionalData = obj.mdfData.(sprintf('%s%s',mdfPrefix,key{1}));
                if ~isempty(mdfOptionalData)
                    lscArgs.(optionalArgMap(key{1})) = mdfOptionalData;
                end
            end
            
            if tfIsSecondaryMotor
                mtrDims = 'z';
            else
                mtrDims = lower(obj.mdfData.(sprintf('%s%s',mdfPrefix,'Dimensions')));
                assert(ischar(mtrDims),'Motor dimensions must be a string.');
                if isempty(mtrDims)
                    mtrDims = 'xyz';
                end
            end
            
            if ~info.NumDimensionsPreset
                if tfIsSecondaryMotor
                    lscArgs.numDeviceDimensions = 1;
                else
                    lscArgs.numDeviceDimensions = length(mtrDims);
                end
            end
            
            % Construct/init LSC
            lscArgsCell = most.util.structPV2cellPV(lscArgs);
            tfErr = false;
            try
                lsc = feval(info.Class,lscArgsCell{:});
                scanimage.components.motors.StageController.initLSC(lsc,mtrDims);
            catch ME
                tfErr = true;
            end
            
            % For common failures (comPort) provide some guidance
            if tfErr
                if ~isfield(lscArgs,'comPort') || isempty(lscArgs.comPort) || ~isnumeric(lscArgs.comPort)
                    ME.rethrow();
                end
                
                portSpec = sprintf('COM%d',lscArgs.comPort);
                
                % check if our ME matches the case of an open port
                if regexp(ME.message,[portSpec ' is not available'])
                    choice = questdlg(['Motor initialization failed because of an existing serial object for ' portSpec ...
                        '; would you like to delete this object and retry initialization?'], ...
                        'Motor initialization error: port Open','Yes','No','Yes');
                    switch choice
                        case 'Yes'
                            % determine which object to delete
                            hToDelete = instrfind('Port',portSpec,'Status','open');
                            delete(hToDelete);
                            disp('Deleted serial object. Retrying motor initialization...');
                            lsc = feval(info.Class,lscArgsCell{:});
                        case 'No'
                            ME.rethrow();
                    end
                else
                    ME.rethrow();
                end
            end
        end
        
        function scObj = ziniMotorStageControllerConstruct(obj,regInfo,lscObj,mdfPrefix)
            scArgs.twoStepEnable = regInfo.TwoStep.Enable;
            if scArgs.twoStepEnable
                
                scArgs.twoStepDistanceThreshold = obj.motorFastMotionThreshold;
                
                % MDF velocity trumps registry velocity. Note that the
                % following may add the field 'velocity' to the
                % FastLSCPropVals, SlowLSCPropVals if it was not there
                % already.
                
                velFast = obj.mdfData.(sprintf('%s%s',mdfPrefix,'VelocityFast'));
                velSlow = obj.mdfData.(sprintf('%s%s',mdfPrefix,'VelocitySlow'));
                if ~isempty(velFast)
                    regInfo.TwoStep.FastLSCPropVals.velocity = velFast;
                end
                if ~isempty(velSlow)
                    regInfo.TwoStep.SlowLSCPropVals.velocity = velSlow;
                end
                
                scArgs.twoStepFastPropVals = regInfo.TwoStep.FastLSCPropVals;
                scArgs.twoStepSlowPropVals = regInfo.TwoStep.SlowLSCPropVals;
                
                %Initialize LSC two-step props to 'slow' values, if specified
                if regInfo.TwoStep.InitSlowLSCProps
                    s = scArgs.twoStepSlowPropVals;
                    props = fieldnames(s);
                    for c=1:numel(props)
                        lscObj.(props{c}) = s.(props{c});
                    end
                end
                
            end
            
            scArgsCell = most.util.structPV2cellPV(scArgs);
            scObj = scanimage.components.motors.StageController(lscObj,scArgsCell{:});
            scObj.moveCompleteDelay = obj.mdfData.moveCompleteDelay;
        end
    end
    
    %%% Abstract method implementation (scanimage.interfaces.Component)
    methods (Access = protected, Hidden)
        function componentStart(~)
            
        end
        
        function componentAbort(~)
            
        end
    end
end

%% LOCAL 
function s = ziniInitPropAttributes()
s = struct();
end


%--------------------------------------------------------------------------%
% Motors.m                                                                 %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
