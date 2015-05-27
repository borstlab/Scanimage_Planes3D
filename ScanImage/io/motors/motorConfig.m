function tf = motorConfig()
%MOTORCONFIG Initialize linear stage controller
%
%% SYNTAX
%   tf = motorConfig()
%       tf: 1 if at least primary motor was successfully configured, 0 otherwise
%
%% NOTES
%   Substantially rewritten on 4/21/11 to handle both primary and secondary controller devices
%
%   This function is designed to never generate an error -- if a configuration error occurs, it displays notification that the motor feature has been disabled
%% CHANGES
%
%% CREDITS
%   Created 4/21/11 by Vijay Iyer
%
%% ******************************************************

global gh state

dimensions = {'X' 'Y' 'Z' 'ZZ'};

if ~isfield(state.motor,'motorOn')
    warning('The ''state.motor.motorOn'' variable was expected, but not found, in the INI file. Motor features are disabled. ');
    state.motor.motorOn = false;
end

%Initialize motor state vars and GUI to default (off) states -- until configuration has succeeded
turnOffMotorButtons();

if ~state.motor.motorOn
    return;
end    
    
%Determine if any motor has been configured
if isempty(state.motor.controllerType)
    if ~isempty(state.motor.controllerTypeZ)
        motorInitError('A secondary z-dimension motor controller was specified without specifying a primary motor controller. This is not supported.');
    end
    
    state.motor.dimensionsAll='none';
    
    for i=1:length(dimensions)
        state.motor.(['resolution' dimensions{i}]) = inf;
    end   
    
    tf = false;
    return;
end

%Determine (intended) dimensional configuration
if isempty(state.motor.dimensions)
    state.motor.dimensions = 'xyz';
else
    state.motor.dimensions = lower(state.motor.dimensions);
end

if isempty(state.motor.controllerTypeZ)
    state.motor.dimensionsAll = state.motor.dimensions;
else
    switch state.motor.dimensions
        case 'xy'
            state.motor.dimensionsAll = 'xy-z';
        case 'xyz'
            state.motor.dimensionsAll = 'xyz-z';
        otherwise
            motorInitError('A secondary Z motor controller can only be used when primary motor is configured for ''xy'' or ''xyz'' dimensions.');
            tf = false;
            return;
    end
end

%Configure primary motor
state.motor.motorOn = configurePrimaryMotor();
if ~state.motor.motorOn
    return;
end

%Configure secondary motor, if primary configuration succeeded
if isfield (state.motor,'controllerTypeZ') && ~isempty(state.motor.controllerTypeZ)
    if configureSecondaryMotor()
        state.motor.motorZOn = true;
    else
        state.motor.motorOn = false; %Disable primary and secondary motor
        return;
    end
end

%Initialize some state vars
state.motor.motorZEnable = strcmpi(state.motor.dimensionsAll,'xy-z');
updateGUIByGlobal('state.motor.motorZEnable');
switch state.motor.dimensionsAll
    case 'none'
        state.motor.dimensionsAllMask = [0 0 0 0];
        disableControlTags = {'cbSecZ' 'pbAltZeroXY' 'pbAltZeroZ' 'etPosZZ'};
    case 'xy'
        state.motor.dimensionsAllMask = [1 1 0 0];
        disableControlTags = {'cbSecZ' 'pbAltZeroXY' 'pbAltZeroZ' 'etPosZZ'};                               
    case {'xyz' 'xy-z'}
        state.motor.dimensionsAllMask = [1 1 1 0];
        disableControlTags = {'cbSecZ' 'pbAltZeroXY' 'pbAltZeroZ' 'etPosZZ'};        
    case 'z'
        state.motor.dimensionsAllMask = [0 0 1 0];
        disableControlTags = {'cbSecZ' 'pbAltZeroXY' 'pbAltZeroZ' 'etPosZZ'};
    case 'xyz-z'
        state.motor.dimensionsAllMask = [1 1 1 1];
        disableControlTags = {'pbZeroXY' 'pbZeroZ'};
        state.motor.dimensionsXYZZ = true;
    otherwise
        assert(false);
end
state.motor.dimensionsAllMask = logical(state.motor.dimensionsAllMask);
state.motor.excludeControls = [state.motor.excludeControls cellfun(@(x)gh.motorControls.(x),disableControlTags)];
set(state.motor.excludeControls,'Visible','off');    

storeResolutionBest();

%Initialize the motorControls GUI
turnOnMotorButtons();

%Bind listeners for motor errors
state.motor.hMotor.addlistener('LSCError',@(src,evnt)motorError);
if state.motor.motorZOn
    state.motor.hMotorZ.addlistener('LSCError',@(src,evnt)motorError);
end

return;

function storeResolutionBest()
%Store resolutionBest, termed 'resolution' in SI3, into per-dimension state
%vars. This is currently only being done for the benefit of the
%scanimage.SI3 class in one place. It would be nicer to remove this
%entirely and solve another way.

global state

resolutionBest = [0 0 0 0];
if state.motor.motorOn
    resolutionBest = state.motor.hMotor.resolutionBest;
    if isscalar(resolutionBest)
        resolutionBest = repmat(resolutionBest,1,3);
    end
end

if state.motor.motorZOn
    resolutionBest(4) = state.motor.hMotorZ.resolutionBest(end); %resolutionBest is either a scalar or 1x3 array
else
    resolutionBest(4) = 0;
end

dimensions = {'X' 'Y' 'Z' 'ZZ'};
for i=1:length(dimensions)
    state.motor.(['resolution' dimensions{i}]) = resolutionBest(i);
end

return;


function tf = configurePrimaryMotor()

global state

tf = false;

%Construct the controller object
try
    
    ctlrInfo = scanimage.MotorRegistry.getControllerInfo(state.motor.controllerType);
    assert(~isempty(ctlrInfo),'Specified controllerType (''%s'') unrecognized or unsupported',state.motor.controllerType); 
    
    pvArgs = {};
    
    %Handle controller sub-type 
    if ~isempty(ctlrInfo.SubType)
        pvArgs = {'controllerType',ctlrInfo.SubType};
    end
    
    %Handle common PV args
    pvArgs = [pvArgs getCommonControllerProps(state.motor.stageType,state.motor.port,state.motor.baud)];
    
    %Handle any INI-file resolution overrides
    resolution = nan(1,3);
    dimensions = {'X' 'Y' 'Z'};
    for i=1:length(dimensions)
        setVal = state.motor.(['resolution' dimensions{i}]);
        if ~isempty(setVal)
            resolution(i) = setVal;
        end
    end
    assert(all(isnan(resolution)) || ~any(isnan(resolution)),'Either none or all of the primary motor resolutionX/Y/Z values must be specified');
    if any(~isnan(resolution))
        pvArgs{end+1} = 'positionDeviceUnits';
        pvArgs{end+1} = resolution * 1e-6;
    end
    
    %Do the actual construction
    hLSC = controllerConstruct(ctlrInfo.Class,pvArgs);
    scanimage.StageController.initLSC(hLSC,state.motor.dimensions);
catch ME
    motorInitError('Error occurred during motor object construction: \n\n%s',most.idioms.reportError(ME));
    return;
end

%Handle post-construction initialization
try    

    ctlrInfo = scanimage.MotorRegistry.getControllerInfo(state.motor.controllerType);
    
    state.motor.hMotor = initializeMotor(hLSC,ctlrInfo,'');
    
    %Signal successful primary motor configuration
    tf = true;
    
catch ME
    motorInitError('Error occurred during motor initialization: \n\t%s',most.idioms.reportError(ME));
    return;
end

return;


function tf = configureSecondaryMotor()

global state

tf = false;

%Construct the secondary Z controller object
try
    ctlrInfo = scanimage.MotorRegistry.getControllerInfo(state.motor.controllerTypeZ);
    
    pvArgs = {};
    
    %Handle controller sub-type
    if ~isempty(ctlrInfo.SubType)
        pvArgs = {'controllerType',ctlrInfo.SubType};
    end
    
    
    pvArgs = getCommonControllerProps(state.motor.stageTypeZ,state.motor.portZ,state.motor.baudZ);
    
    if ~isempty(state.motor.resolutionZZ)
        pvArgs{end+1} = 'positionDeviceUnits';
        pvArgs{end+1} = state.motor.resolutionZZ * 1e-6;
    end    

    %Do the actual construction
    hLSC = controllerConstruct(ctlrInfo.Class,pvArgs);
    scanimage.StageController.initLSC(hLSC,'z');
catch ME
    motorInitError('Error occurred during secondary motor object construction: \n\t%s',most.idioms.reportError(ME));
    return;
end

%Initialize the secondary Z controller object
try
    %hMotorZ = state.motor.hMotorZ;
    %hMotorZ.initialize();
    
    state.motor.hMotorZ = initializeMotor(hLSC,ctlrInfo,'Z');
    
    %Signal that initialization has completed successfully
    tf = true;
    
catch ME
    motorInitError('Error occurred during secondary motor object initialization: \n\t%s',most.idioms.reportError(ME));
    return;
end

return;


function hMotor = initializeMotor(hLSC,ctlrInfo,paramSuffix)

global state

timeoutParam = ['state.motor.timeout' paramSuffix];
timeoutVal = eval(timeoutParam);
if ~isempty(timeoutVal)
    assert(isnumeric(timeoutVal) && timeoutVal > 0, 'Invalid value for ''%s''',timeoutParam);
    hLSC.moveTimeout = timeoutVal; 
end
%     hMotor.asyncMoveTimeout = state.motor.(['timeout' paramSuffix]); %TODO: Clarify in INI file that this is a MOVE timeout, not timeout for standard serial port command/reply pairs
%     if isempty(hMotor.asyncMoveTimeout)
%         hMotor.moveTimeout = state.motor.timeout;
%     end
%

%Enable twoStepMoveEnable, where appropriate
%Set moveMode/resolutionMode values for various controller types (if omitted, 'default' is used)
twoStepEnable = ctlrInfo.TwoStep.Enable;
if twoStepEnable
    twoStepProps.Fast = ctlrInfo.TwoStep.FastLSCPropVals;
    twoStepProps.Slow = ctlrInfo.TwoStep.SlowLSCPropVals;
    
    %Set default fast & slow velocities, as required if not given in INI file
    twoStepSpeeds = {'Fast' 'Slow'};
    for i=1:length(twoStepSpeeds)
        speed = twoStepSpeeds{i};
		
		fieldName = ['velocity' speed paramSuffix];
        
        if isnan(hLSC.velocity)
            state.motor.(fieldName) = nan;
        elseif ~isempty(state.motor.(fieldName))
            twoStepProps.(speed).velocity = state.motor.(fieldName);
        else
            switch state.motor.(['controllerType' paramSuffix])
                case {'mp285' 'sutter.mp285'}
                    hLSC.resolutionMode = twoStepProps.(speed).resolutionMode;
                    twoStepProps.(speed).velocity = hLSC.maxVelocity;
                    
                case {'scientifica'}
                    if strcmpi(speed,'fast')
                        twoStepProps.(speed).velocity = hLSC.maxVelocity;
                    else %'slow'
                        twoStepProps.(speed).velocity = max(round(twoStepProps.Fast.velocity / 8),1);
                    end
                    
            end
		end
    end
    
    %Initialize property values to two-step 'slow' values, if indicated
    if ctlrInfo.TwoStep.InitSlowLSCProps
        propNames = fieldnames(twoStepProps.Slow);
        
        for c=1:numel(propNames)
            hLSC.(propNames{c}) = twoStepProps.Slow.(propNames{c});
        end               
    end
        
else
    twoStepProps.Fast = struct();
    twoStepProps.Slow = struct();
end

%Construct the scanimage.StageController object
hMotor = scanimage.StageController(hLSC,'twoStepEnable',twoStepEnable,...
    'twoStepSlowPropVals',twoStepProps.Slow,...
    'twoStepFastPropVals',twoStepProps.Fast,...
    'twoStepDistanceThreshold',state.motor.(['fastMotionThreshold' paramSuffix]));

% %Set the position resolution
% posnResolution = state.motor.(['posnResolution' paramSuffix]);
%TODO!!
% if ~isempty(posnResolution)
%     hMotor.hLSC.moveVerificationPositionTolerance = posnResolution;
% end

if ~isempty(timeoutVal)
    hMotor.nonblockingMoveTimeout = timeoutVal; 
end

%Identify if reset is safe
state.motor.(sprintf('safeReset%s',paramSuffix)) = ctlrInfo.SafeReset;


return;


function pvArgs = getCommonControllerProps(stageTypeSpec, portSpec, baudSpec)

pvArgs = {};

if ~isempty(stageTypeSpec)
    pvArgs{end+1} = 'stageType';
    pvArgs{end+1} = stageTypeSpec;
end

if ~isempty(portSpec)
    if isnumeric(portSpec) && isscalar(portSpec)
        comPort = portSpec;
    elseif ischar(portSpec)
        comPort = str2num(strtok(lower(portSpec),'com'));
    else
        assert(false,'Unrecognized COM port specification');
    end
    
    pvArgs{end+1} = 'comPort';
    pvArgs{end+1} = comPort;
end

if ~isempty(baudSpec)
    pvArgs{end+1} = 'baudRate';
    pvArgs{end+1} = baudSpec;
end

return;

function hLSC = controllerConstruct(controllerClass,pvArgs)

try
    hLSC = feval(controllerClass,pvArgs{:});
catch ME
    
    pvArgStruct = struct(pvArgs{:});
    
    if ~isfield(pvArgStruct,'comPort') || isempty(pvArgStruct.comPort)
        ME.rethrow();
    end
    
    if isnumeric(pvArgStruct.comPort)
        portSpec = sprintf('COM%d',pvArgStruct.comPort);
    end
    
    % check if our ME matches the case of an open port
    if regexp(ME.message,[portSpec ' is not available'])
        try
            choice = questdlg(['Motor initialization failed because of an existing serial object for ' portSpec ...
                '; would you like to delete this object and retry initialization?'], ...
                'Motor initialization error: port Open','Yes','No','Yes');
            switch choice
                case 'Yes'
                    % determine which object to delete
                    hToDelete = instrfind('Port',portSpec,'Status','open');
                    delete(hToDelete);
                    clear hToDelete;
                    
                    disp('Deleted serial object. Retrying motor initialization...');
                    hStage = feval(controllerClass,pvArgs{:});
                case 'No'
                    ME.rethrow();
            end
        catch ME
            ME.rethrow();
        end
    else
        ME.rethrow();
    end
end

return;






function motorInitError(errorMessage,varargin)
%This function never generates an error -- simply display notification of motor initialation error

fprintf(2,'***** Motor controller feature has been disabled, due to error during initialization:\n%s\n*****\n',sprintf(errorMessage,varargin{:}));
%disableMotor(); %VI112310B

return;







