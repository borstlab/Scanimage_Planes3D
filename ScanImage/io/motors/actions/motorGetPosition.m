%% function [posnAbsolute,posnRelative] = motorGetPosition()
% Function that will read the position from the motor controllers and update the ScanImage state variables (and screen display)
%
%% SYNTAX
%   posnAbsolute: Output of current position value read from motor controller, in absolute coordinates.
%   posnRelative: Output of current position value read from motor controller, in relative coordinates.
%
%% NOTES
%   Function rewritten from scratch, and renamed from updateMotorPosition(). See updateMotorPosition.mold for original version.
%
%   motorGetPosition() combines several operations:
%       1) Retrieve position from motor controller (in absolute coordinates)
%       2) Display position to MotorControls GUI, always in /relative/ coordinates
%       3) Update /both/ ScanImage absolute and relative position state variables
%   
%   Then the absolute and relative position are returned, if needed.
%
%   An alternative might be to have an input argument determine whether absolute or relative position is returned, or have two separate functions.
%   However, at this time, there is no use case in ScanImage where posnRelative is required as a return value.
%   In other words, at this time, ScanImage always displays relative position but internally uses absolute position for cases where positions are stored for particular operations. -- Vijay Iyer 6/6/10
%
%   When XYZ-Z dimension mode is configured (in INI file), this function returns/sets 4-vector position values
%
%% CREDITS
%   Created 3/23/10, by Vijay Iyer. Based on original updateMotorPosition() by Tom Pologruto, 2/1/01.
%% **************************

function [posnAbsolute, posnRelative] = motorGetPosition()


global state  

if ~state.motor.motorOn 
    if nargout == 0
        return;
    else
        error('Motor action attempted, but no motor has been configured');
    end
end

posnAbsolute = state.motor.hMotor.positionAbsolute; %Reads position from motor controller, in absolute coordinates
if isempty(posnAbsolute)    
    if state.motor.hMotor.lscErrPending %should always be the case
        error('ScanImage:MotorGetPosnError','Error occurred while reading motor position'); 
    end
end


%Read secondary Z value
if state.motor.motorZOn
    posnAbsoluteZ = state.motor.hMotorZ.positionAbsolute;
    
    if isempty(posnAbsoluteZ)
        if state.motor.hMotorZ.lscErrPending %should always be the case
            error('ScanImage:MotorGetPosnError','Error occurred while reading motor position');
        end
    end
end

%Update posn state vars
state.motor.absXPosition = posnAbsolute(1);
state.motor.absYPosition = posnAbsolute(2);

if ~state.motor.motorZOn
    state.motor.absZPosition = posnAbsolute(3);
    state.motor.lastPositionRead = posnAbsolute;
else
    if state.motor.dimensionsXYZZ
        state.motor.absZPosition = posnAbsolute(3);
        state.motor.absZZPosition = posnAbsoluteZ(3);
        state.motor.lastPositionRead = [posnAbsolute posnAbsoluteZ(3)];
        
        posnAbsolute = state.motor.lastPositionRead;
    else
        state.motor.absZPosition = posnAbsoluteZ(3);
        state.motor.lastPositionRead = [posnAbsolute(1:2) posnAbsoluteZ(3)];
        
        posnAbsolute = state.motor.lastPositionRead;        
    end
end

%Update the relative position state variables
motorUpdatePositionDisplay();

%Update posnRelative output variable, if needed
if nargout > 1
    if state.motor.dimensionsXYZZ
        posnRelative = zeros(1,4);
        posnRelative(4) = state.motor.relZZPosition;
    else
        posnRelative = zeros(1,3);
    end
    posnRelative(1) = state.motor.relXPosition;
    posnRelative(2) = state.motor.relYPosition;
    posnRelative(3) = state.motor.relZPosition;
end

notify(state.hSI,'motorPositionUpdated');
	
end
