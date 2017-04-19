% Most Software Machine Data File

%% ScanImage

%Scanner systems
scannerNames = {'LinScanner'};      % Cell array of string names for each scan path in the microscope
scannerTypes = {'Linear'};        % Cell array indicating the type of scanner for each name. Current options: {'Resonant' 'Linear}

%Simulated mode
simulated = false;                  % Boolean for activating simulated mode. For normal operation, set to 'false'. For operation without NI hardware attached, set to 'true'.

%Optional components
components = {};                    % Cell array of optional components to load. Ex: {'dabs.thorlabs.ECU1' 'dabs.thorlabs.BScope2'}

%Data file location
dataDir = '[MDF]\ConfigData';       % Directory to store persistent configuration and calibration data. '[MDF]' will be replaced by the MDF directory

%% Shutters
%Shutter(s) used to prevent any beam exposure from reaching specimen during idle periods. Multiple
%shutters can be specified and will be assigned IDs in the order configured below.
shutterDaqDevices = {'Dev5'};  % Cell array specifying the DAQ device or RIO devices for each shutter eg {'PXI1Slot3' 'PXI1Slot4'}
shutterChannelIDs = {'PFI0'};      % Cell array specifying the corresponding channel on the device for each shutter eg {'port0/line0' 'PFI12'}

shutterOpenLevel = 1;               % Logical or 0/1 scalar indicating TTL level (0=LO;1=HI) corresponding to shutter open state for each shutter line. If scalar, value applies to all shutterLineIDs
shutterOpenTime = 0.1;              % Time, in seconds, to delay following certain shutter open commands (e.g. between stack slices), allowing shutter to fully open before proceeding.

%% Beams
beamDaqDevices = {};                            % Cell array of strings listing beam DAQs in the system. Each scanner set can be assigned one beam DAQ ex: {'PXI1Slot4'}

% Define the parameters below for each beam DAQ specified above, in the format beamDaqs(N).param = ...
beamDaqs(1).modifiedLineClockIn = '';           % one of {PFI0..15, ''} to which external beam trigger is connected. Leave empty for automatic routing via PXI/RTSI bus
beamDaqs(1).frameClockIn = '';                  % one of {PFI0..15, ''} to which external frame clock is connected. Leave empty for automatic routing via PXI/RTSI bus
beamDaqs(1).referenceClockIn = '';              % one of {PFI0..15, ''} to which external reference clock is connected. Leave empty for automatic routing via PXI/RTSI bus
beamDaqs(1).referenceClockRate = 10e6;          % if referenceClockIn is used, referenceClockRate defines the rate of the reference clock in Hz. Default: 10e6Hz

beamDaqs(1).chanIDs = [];                       % Array of integers specifying AO channel IDs, one for each beam modulation channel. Length of array determines number of 'beams'.
beamDaqs(1).displayNames = {};                  % Optional string cell array of identifiers for each beam
beamDaqs(1).voltageRanges = 1.5;                % Scalar or array of values specifying voltage range to use for each beam. Scalar applies to each beam.

beamDaqs(1).calInputChanIDs = [];               % Array of integers specifying AI channel IDs, one for each beam modulation channel. Values of nan specify no calibration for particular beam.
beamDaqs(1).calOffsets = [];                    % Array of beam calibration offset voltages for each beam calibration channel
beamDaqs(1).calUseRejectedLight = false;        % Scalar or array indicating if rejected light (rather than transmitted light) for each beam's modulation device should be used to calibrate the transmission curve 
beamDaqs(1).calOpenShutterIDs = [];             % Array of shutter IDs that must be opened for calibration (ie shutters before light modulation device).

%% LinScan (LinScanner)
deviceNameAcq   = 'Dev1';      % string identifying NI DAQ board for PMT channels input
deviceNameGalvo = 'Dev5';      % string identifying NI DAQ board for controlling X/Y galvo. leave empty if same as deviceNameAcq
deviceNameAux   = 'Dev5';      % string identifying NI DAQ board for outputting clocks. leave empty if unused. Must be a X-series board

%Optional
channelsInvert = false;             % scalar or vector identifiying channels to invert. if scalar, the value is applied to all channels
beamDaqID = [];                     % Numeric: ID of the beam DAQ to use with the linear scan system
shutterIDs = 1;                     % Array of the shutter IDs that must be opened for linear scan system to operate

resScanframeClockIn = '';           % one of {'',PFI8} to which frame clock from ResScan is connected on Aux board. Leave empty for automatic routing via PXI bus
referenceClockIn = '';              % one of {'',PFI14} to which 10MHz reference clock is connected on Aux board. Leave empty for automatic routing via PXI bus

enableRefClkOutput = false;         % Enables/disables the export of the 10MHz reference clock on PFI14
%Scanner control
XMirrorChannelID = 0;               % The numeric ID of the Analog Output channel to be used to control the X Galvo.
YMirrorChannelID = 1;               % The numeric ID of the Analog Output channel to be used to control the y Galvo.
ZMirrorChannelID = 2;               % The numeric ID of the Analog Output channel to be used to control the Z piezo.

xGalvoAngularRange = 15;            % max range in optical degrees (pk-pk) for x galvo
yGalvoAngularRange = 15;            % max range in optical degrees (pk-pk) for y galvo
zPiezoRange = 280;                  % max range in microns for z piezo

scanParkAngleX = 0;              % Numeric [deg]: Optical degrees from center position for X galvo to park at when scanning is inactive
scanParkAngleY = 0;              % Numeric [deg]: Optical degrees from center position for Y galvo to park at when scanning is inactive
scanParkAngleZ = 0;              % Numeric [microns]: Microns from center position for Z piezo to park when scanning is inactive [feature currently not implemented]

voltsPerOpticalDegreeX = 0.5;         % galvo conversion factor from optical degrees to volts (negative values invert scan direction)
voltsPerOpticalDegreeY = 0.5;         % galvo conversion factor from optical degrees to volts (negative values invert scan direction)
opticalDegreesPerMicronXY = 0.0442;   % conversion factor from Galvo optical degrees to microns in the sample (measure empirically with micromanipulator or a sample of known size)
voltsPerMicronZ = 0.0595;             % conversion factor from zPiezo volts to microns depth in the sample. Theoretical value for this setup (depends on magnification and immersion media) is 0.0476. Empirical value is slightly higher and, the way it was measured, rather variable. Higher by 1/6 to 1/3 of the theoretical value. For now added 1/4 to the theoretical value (0.0595). 
dummyValueZ= 0;                       % necessary atm because Piezo and Galvos are the same class, change at some point

scanOffsetAngleX = -6;               % angle in optical degrees to shift command waveform applied to X-scanner
scanOffsetAngleY = 0;               % angle in optical degrees to shift command waveform applied to Y-scanner
scanOffsetZ = 0;                    % in microns in object (not of piezo), to shift command waveform applied to Z-scanner. Large offsets better done by manually moving the piezo stage. 

%Acquisition
channelIDs = [];                    % Array of numeric channel IDs for PMT inputs. Leave empty for default channels (AI0...AIN-1)

%% Motors
%Motor used for X/Y/Z motion, including stacks. 
%motorDimensions & motorControllerType must be specified to enable this feature.
motorControllerType = '';           % If supplied, one of {'sutter.mp285', 'sutter.mpc200', 'thorlabs.mcm3000', 'thorlabs.mcm5000', 'scientifica', 'pi.e665', 'pi.e816', 'npoint.lc40x'}.
motorDimensions = '';               % If supplied, one of {'XYZ', 'XY', 'Z'}. Defaults to 'XYZ'.                
motorStageType = '';                % Some controller require a valid stageType be specified
motorCOMPort = [];                  % Integer identifying COM port for controller, if using serial communication
motorBaudRate = [];                 % Value identifying baud rate of serial communication. If empty, default value for controller used.
motorZDepthPositive = true;         % Logical indicating if larger Z values correspond to greater depth
motorPositionDeviceUnits = [];      % 1x3 array specifying, in meters, raw units in which motor controller reports position. If unspecified, default positionDeviceUnits for stage/controller type presumed.
motorVelocitySlow = [];             % Velocity to use for moves smaller than motorFastMotionThreshold value. If unspecified, default value used for controller. Specified in units appropriate to controller type.
motorVelocityFast = [];             % Velocity to use for moves larger than motorFastMotionThreshold value. If unspecified, default value used for controller. Specified in units appropriate to controller type.

%Secondary motor for Z motion, allowing either XY-Z or XYZ-Z hybrid configuration
motor2ControllerType = '';          % If supplied, one of {'sutter.mp285', 'sutter.mpc200', 'thorlabs.mcm3000', 'thorlabs.mcm5000', 'scientifica', 'pi.e665', 'pi.e816', 'npoint.lc40x'}.
motor2StageType = '';               % Some controller require a valid stageType be specified
motor2COMPort = [];                 % Integer identifying COM port for controller, if using serial communication
motor2BaudRate = [];                % Value identifying baud rate of serial communication. If empty, default value for controller used.
motor2ZDepthPositive = true;        % Logical indicating if larger Z values correspond to greater depth
motor2PositionDeviceUnits = [];     % 1x3 array specifying, in meters, raw units in which motor controller reports position. If unspecified, default positionDeviceUnits for stage/controller type presumed.
motor2VelocitySlow = [];            % Velocity to use for moves smaller than motorFastMotionThreshold value. If unspecified, default value used for controller. Specified in units appropriate to controller type.
motor2VelocityFast = [];            % Velocity to use for moves larger than motorFastMotionThreshold value. If unspecified, default value used for controller. Specified in units appropriate to controller type.

%Global settings that affect primary and secondary motor
moveCompleteDelay = 0;              % Numeric [s]: Delay from when stage controller reports move is complete until move is actually considered complete. Allows settling time for motor

%% FastZ
%FastZ hardware used for fast axial motion, supporting fast stacks and/or volume imaging
%fastZControllerType must be specified to enable this feature. 
%Specifying fastZControllerType='useMotor2' indicates that motor2 ControllerType/StageType/COMPort/etc will be used.
fastZControllerType = '';           % If supplied, one of {'useMotor2', 'pi.e709', 'pi.e753', 'pi.e665', 'pi.e816', 'npoint.lc40x', 'analog'}. 
fastZCOMPort = [];                  % Integer identifying COM port for controller, if using serial communication
fastZBaudRate = [];                 % Value identifying baud rate of serial communication. If empty, default value for controller used.

%Some FastZ hardware requires or benefits from use of an analog output used to control sweep/step profiles
%If analog control is used, then an analog sensor (input channel) must also be configured
fastZDeviceName = '';               % String specifying device name used for FastZ control
frameClockIn = '';                  % One of {PFI0..15, ''} to which external frame trigger is connected. Leave empty for automatic routing via PXI/RTSI bus
fastZAOChanID = [];                 % Scalar integer indicating AO channel used for FastZ control
fastZAIChanID = [];                 % Scalar integer indicating AI channel used for FastZ sensor

%% LSC Pure Analog
commandVoltsPerMicron = []; % Conversion factor for command signal to analog linear stage controller
sensorVoltsPerMicron = [];  % Conversion signal for sensor signal from analog linear stage controller. Leave empty for automatic calibration

commandVoltsOffset = 0; % Offset value, in volts, for command signal to analog linear stage controller
sensorVoltsOffset = 0;  % Offset value, in volts, for sensor signal from analog linear stage controller. Leave empty for automatic calibration

% Optional limits (any of these fields can be left blank; if ommited, default limits are +/-10V)
maxCommandVolts = [];       % Maximum allowable voltage command
maxCommandPosn = [];        % Maximum allowable position command in microns
minCommandVolts = [];       % Minimum allowable voltage command
minCommandPosn = [];        % Minimum allowable position command in microns

analogCmdBoardID = ''; % String specifying NI board identifier (e.g. 'Dev1') containing AO channel for LSC control
analogCmdChanIDs = []; % Scalar indicating AO channel number (e.g. 0) used for analog LSC control
analogSensorBoardID = ''; % String specifying NI board identifier (e.g. 'Dev1') containing AI channel for LSC position sensor
analogSensorChanIDs = []; % Scalar indicating AI channel number (e.g. 0) used for analog LSC position sensor

