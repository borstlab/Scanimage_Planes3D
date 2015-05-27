%% function relOrigin = motorGetRelativeOrigin()
% Function that retrieves the origin of the relative position coordinates 
%
%% SYNTAX
%   relOrigin: Numeric 1x3 array specifying position of relative origin, in absolute coordinates 
%
%% NOTES
%   Makes use of the relativeOrigin property maintained by the LinearStageControllerClass
%
%% CREDITS
%  Created 3/23/10, by Vijay Iyer
%% *********************
function relOrigin = motorGetRelativeOrigin()

global state 

if ~state.motor.motorOn 
    if nargout == 0
        return;
    else
        error('Motor action attempted, but no motor has been configured');
    end
end

if state.motor.motorZOn
    relOriginZ = state.motor.hMotorZ.relativeOrigin(3);
    
    if state.motor.dimensionsXYZZ
        relOrigin = [state.motor.hMotor.relativeOrigin relOriginZ];
    else
        relOrigin = [state.motor.hMotor.relativeOrigin(1:2) relOriginZ];
    end
else
    relOrigin = state.motor.hMotor.relativeOrigin;       
end


