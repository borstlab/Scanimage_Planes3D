function motorSetPosition(coordType,newPos,displayMode)
%% function motorSetPosition(coordType,newPos,displayMode)
% Generic function for commanding a motor move operation to newly specified coordinates

%% SYNTAX
%  motorSetPosition(coordType)
%  motorSetPosition(coordType,newPos)
%  motorSetPosition(coordType,newPos,displayMode)
%  motorSetPosition(coordType,newPos,displayMode,forceSecZ)
%       coordType: One of {'relative' 'absolute'} specifying which coordinates set to use
%       newPos: <OPTIONAL> 1x3 or 1x4 array of absolute X/Y/Z positions to which motor should go. If empty, the absX/Y/ZPosition state variable is used. 
%       displayMode: <OPTIONAL;DEFAULT='none'> One of {'assume' 'verify' 'none'}.  Determines now newly set position should be displayed in ScanImage:
%                       'assume': The ScanImage relative/absolute X/Y/Z positions (those displayed) are updated to match the specified position at start of move
%                       'verify': Same as 'assume', but motorGetPosition() is called at end of move which updates ScanImage stored/displayed position values to match that read from device.
%                       'none': Do not set the Scanimage stored/displayed relative/absolute X/Y/Z positions and do not explicitly retrieve the final position from the motor controller
%       forceSecZ: <Optional,Default=false> If true, then secondary z controller is used regardless of state.motor.motorZEnable setting -- Vijay Iyer 5/12/11
%
%% NOTES
%   Depending on state.motor.motorEnable & state.motor.dimensionsXYZZ, move may involve either 1 or 2 scanimage.StageController objects
%
%   VI 04/26/2011: It appears there's a bug in handling the 'none' case in the case where new coordinates are given -- but this case may never be actually used by SI
%
%% CHANGES
%   VI032010A: Use motorCheckPendingMove() to check/handle pending move, if any. -- Vijay Iyer 3/20/10
%   VI052010A: Add 'displayMode' argument and handling -- Vijay Iyer 5/20/10
%   VI120910A: Remove state.motor.movePending altogether -- Vijay Iyer 12/9/10
%   VI042511A: Should not need to ever change twoStepMoveEnable property after initial motor configuration -- Vijay Iyer 4/25/11
%
%% CREDITS
%  Created 3/13/10, by Vijay Iyer
%  Based on original setMotorPosition.m written by Tom Pologruto & Bernardo Sabatini, 01/2001
%
%% *********************

global state

if ~state.motor.motorOn
    return;
end

hMotor = state.motor.hMotor;
hMotorZ = state.motor.hMotorZ;
 
%%%VI120910A: Removed%%%
%%Check if move is pending and try interrupt, if so. 
%motorCheckPendingMove(); %VI032010A

%If position is not supplied, then use state.motor.<rel/abs>X/Y/ZPosition variables
useStateVarCoords = nargin < 2 || isempty(newPos);
if nargin < 3 || isempty(displayMode)
    displayMode = 'none';
end
    

switch coordType
    case 'absolute'  
        moveCompleteFcn = @hMotor.moveCompleteAbsolute;
        
        if useStateVarCoords
            newPos(1) = state.motor.absXPosition;		% Set X Position to new value
            newPos(2) = state.motor.absYPosition;		% Set Y Position to new value
            newPos(3) = state.motor.absZPosition;		% Set Z Position to new value
            
            if state.motor.dimensionsXYZZ
                newPos(4) = state.motor.absZZPosition;
            end
        %%%VI052010A%%%%%%%%%%%
        elseif ismember(displayMode,{'assume' 'verify'})
            state.motor.absXPosition = newPos(1);
            state.motor.absYPosition = newPos(2);
            state.motor.absZPosition = newPos(3);
            
            if length(newPos) == 4 && state.motor.dimensionsXYZZ
                state.motor.absZZPosition = newPos(4);
            end

            motorUpdatePositionDisplay();
        end
        %%%%%%%%%%%%%%%%%%%%%%%        
        
        moveCompleteFcnZ = @hMotorZ.moveCompleteAbsolute;
                
    case 'relative'
        moveCompleteFcn = @hMotor.moveCompleteRelative;
        
        if useStateVarCoords
            newPos(1) = state.motor.relXPosition;		% Set X Position to new value
            newPos(2) = state.motor.relYPosition;		% Set Y Position to new value
            newPos(3) = state.motor.relZPosition;		% Set Z Position to new value
            
            if state.motor.dimensionsXYZZ
                newPos(4) = state.motor.relZZPosition;
            end
            
        %%%VI052010A%%%%%%%%%%%
        elseif ismember(displayMode,{'assume' 'verify'})
            relOrigin = motorGetRelativeOrigin();
            
            state.motor.absXPosition = newPos(1) + relOrigin(1);
            state.motor.absYPosition = newPos(2) + relOrigin(2);
            state.motor.absZPosition = newPos(3) + relOrigin(3);
            
            if length(newPos) == 4 && state.motor.dimensionsXYZZ
                state.motor.absZZPosition = newPos(4) + relOrigin(4);
            end
            
            motorUpdatePositionDisplay();
        end
        %%%%%%%%%%%%%%%%%%%%%%%
                        
        moveCompleteFcnZ = @hMotorZ.moveCompleteRelative;
    otherwise
        error('Value of ''coordType'' must be ''relative'' or ''absolute''');
end

%The move action
setStatusString('Moving stage...'); %it might take a few moments
turnOffMotorButtons;
%Set velocity/resolutionMode/moveMode to 'fast' counterparts, while enabling two-step move -- the pre-cached slow values will be used to finalize move

% set(hMotor,'twoStepMoveEnable',state.motor.twoStepMoveAllow); %VI042511A: Removed
% DEQ20101215
% set(hMotor,'twoStepMoveEnable',state.motor.twoStepMoveAllow,'resolutionMode',state.motor.resolutionModeFast,...
%     'moveMode',state.motor.moveModeFast,'velocity',state.motor.velocityFast); %VI0321010A: Mimicking original ScanImage behavior -- use two-step moves for move operations outside of a stack context


if state.motor.dimensionsXYZZ %XYZ-Z: move 1 or 2 motors to specified 3 or 4-vector coordinates, regardless of motorZEnable state
    moveCompleteFcn(newPos(1:3));
    if length(newPos) > 3
        moveCompleteFcnZ([nan nan newPos(4)]);
    end
elseif state.motor.motorZEnable %XY-Z: move both motors to specified 3-vector coordinates 
    moveCompleteFcn([newPos(1:2) nan]);
    moveCompleteFcnZ([nan nan newPos(3)]);
else %Move primary motor to specified 3-vector coordinates (whether XYZ, XY, or Z)
    moveCompleteFcn(newPos(1:3)); 
end

%%%VI052010A%%%%%%%%%%%
if strcmpi(displayMode,'verify')
    motorGetPosition(); %This is a 'nested' motor action -- need not be wrapped by motorAction()
    state.motor.lastPositionSet = state.motor.lastPositionRead;
else
    state.motor.lastPositionSet = newPos;
    notify(state.hSI,'motorPositionUpdated');
end
    
%%%%%%%%%%%%%%%%%%%%%%%

turnOnMotorButtons;
setStatusString('');





