%% function motorUpdatePositionDisplay(relPosn)
% Updates position display on Motor Controls window -- the relative coordinates, distance, and stack endpoints
%
%% SYNTAX
%   relPosn: <OPTIONAL> New relative position to display in ScanImage. 
%       If not supplied, the relative position will be determined from the current /absolute/ position state variables (state.motor.abs<X,Y,Z>Position) 
%
%% NOTES
%   Function created from scratch, replacing updateRelativeMotorPosition. To see original, see updateRelativeMotorPosition.MOLD.
%
%   Function not only updates ScanImage relative position display, but also updates relative position state variables.
%   If relPosn argument is supplied, then absolute position state variables are updated, in addition to relative position state variables.
%
%   In this manner, the absolute and relative position state variables are always correctly linked to each other.
%
%   Note this function simply updates display and state variables -- it /does not/ actually move or check the motor position
%   As a result, it is /not/ a motor 'action'
%
%% CHANGES
%   VI051111A: Ensure that inactive dimensions always are displayed as NaN -- Vijay Iyer 5/11/11
%   VI051711A: Display distance correctly regardless of number of dimensions in relative position vector (1,2,3, or 4 vector) -- Vijay Iyer 5/17/11
%   
%% CREDITS
%   Created 3/24/10, by Vijay Iyer. Based on original updateRelativeMotorPosition().
%% *********************************************************

 function motorUpdatePositionDisplay(relPosn)

 global state
 
 if isempty(state.motor.hMotor)
     return;
 end
 
 relOrigin = state.motor.hMotor.relativeOrigin;
 
 if state.motor.motorZOn
     if state.motor.dimensionsXYZZ
         relOrigin(4) = state.motor.hMotorZ.relativeOrigin(3);
     else
         relOrigin(3) = state.motor.hMotorZ.relativeOrigin(3);
     end
 end

 
 if nargin < 1 || isempty(relPosn)
     state.motor.relXPosition = state.motor.absXPosition - relOrigin(1);
     state.motor.relYPosition = state.motor.absYPosition - relOrigin(2);
     
     if state.motor.dimensionsXYZZ
         state.motor.relZPosition = state.motor.absZPosition - relOrigin(3);
         state.motor.relZZPosition = state.motor.absZZPosition - relOrigin(4);         
     else
         state.motor.relZPosition = state.motor.absZPosition - relOrigin(3);
     end
     
      relPosn = [state.motor.relXPosition state.motor.relYPosition state.motor.relZPosition state.motor.relZZPosition]; %VI051711A

 else
     assert(isnumeric(relPosn) && isvector(relPosn) && ismember(length(relPosn),[3 4]),'The optional ''relPosn'' argument must be a 3-element vector specifing relative position');
     
     relPosn(~state.motor.dimensionsAllMask) = nan; %VI051111A
     
     state.motor.relXPosition = relPosn(1);
     state.motor.relYPosition = relPosn(2);
     state.motor.relZPosition = relPosn(3);
  
     state.motor.absXPosition = state.motor.relXPosition + relOrigin(1);
     state.motor.absYPosition = state.motor.relYPosition + relOrigin(2);
     state.motor.absZPosition = state.motor.relZPosition + relOrigin(3);     
     
     if state.motor.dimensionsXYZZ
         state.motor.relZZPosition = relPosn(4);
         state.motor.absZZPostion = state.motor.relZZPosition + relOrigin(4); 
     end

 end     
 
 %%%VI051711A%%%
 %state.motor.distance = sqrt(state.motor.relXPosition^2+state.motor.relYPosition^2+state.motor.relZPosition^2+state.motor.relZZPosition^2); %VI051711A: Removed
 relPosn(isnan(relPosn)) = 0;
 state.motor.distance = norm(relPosn);
 %%%%%%%%%%%%%%%%
 
 updateGUIByGlobal('state.motor.relXPosition');
 updateGUIByGlobal('state.motor.relYPosition');
 updateGUIByGlobal('state.motor.relZPosition');
 updateGUIByGlobal('state.motor.relZZPosition');
 updateGUIByGlobal('state.motor.distance');
 
 updateStackEndpoints(relOrigin); %Provide relative origin, since it's been determined already
	
	
