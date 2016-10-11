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
