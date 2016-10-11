%% function motorSetRelativeOrigin(dimArray)
% Function that updates the origin of the relative position coordinates to match current position, in all or selected dimensions
%
%% SYNTAX
%   dimArray: (OPTIONAL) Logical 1x3 or 1x4 array specifying which dimensions to update relative origin for. If empty/omitted, [1 1 1] is assumed. 
%
%% NOTES
%   Makes use of the relativeOrigin property maintained by the LinearStageControllerClass
%
%% CHANGES
%   VI060610A: Use motorUpdatePositionDisplay(), in lieu of motorGetPosition(), to update ScanImage position display without additional call to motor interface -- Vijay Iyer 6/6/10
%   VI051211A: BUGFIX - Call motorUpdatePositionDisplay() without relPosn argument, which was causing zero XY/Z operations to appear as XYZ -- Vijay Iyer 5/12/11
%   VI121011A: Call motorGetPosition() after setting relative origin to ensure ScanImage position state vars are up to date. This is a redundant read with zeroSoft(), but not consequential. -- Vijay Iyer 12/10/11
%
%% CREDITS
%  Created 3/23/10, by Vijay Iyer
%% *********************
function motorSetRelativeOrigin(dimArray)

global state 

if ~state.motor.motorOn
    return;
end

turnOffMotorButtons;

switch state.motor.dimensionsAll
    case {'xy' 'z' 'xyz'}
        state.motor.hMotor.zeroSoft(dimArray(1:3)); %Reads current position and sets it as origin
   
    case {'xy-z'}
        if any(dimArray(1:2))
            state.motor.hMotor.zeroSoft([dimArray(1:2) 0]);
        end
        
        if dimArray(3)
            state.motor.hMotorZ.zeroSoft([0 0 dimArray(3)]);
        end
        
    case {'xyz-z'}
        if any(dimArray(1:3))
            state.motor.hMotor.zeroSoft(dimArray(1:3))
        end
        
        if length(dimArray) >= 4 && dimArray(4)
            state.motor.hMotorZ.zeroSoft([0 0 dimArray(4)]);
        end
end    

motorGetPosition(); %VI121011A
%motorUpdatePositionDisplay(); %VI121011A: Removed %VI051211A %VI060610A
turnOnMotorButtons;

if ~state.hSI.roiShowAbsoluteCoords
	state.hSI.roiUpdatePositionTable();
end

% % update the RDF
% state.hSI.roiUpdateShownPosition();