function motorPositionDefine(positionNumber)
%MOTORPOSITIONDEFINE Define new motor position at current motor coordinates
%
%% SYNTAX
%   positionNumber: Identifies the stored motor position number to assign the current motor coordinates. If empty/omitted, the current value of state.motor.position is used
%   
%% NOTES
%   Function replaces definePosition().
%   
%   VI 04/26/2011 - Decided, for now, to not support 4-vector position save/load operation
%
%% CREDITS
%   Created 3/26/10, by Vijay Iyer. Based on original function definePosition.
%% **************************

global state

if (nargin<1)
    positionNumber=state.motor.position; %Get position number from GUI-bound state variable
end

if isempty(state.motor.positionVectors)
    state.motor.positionVectors=zeros(positionNumber, 3);
end

%Read the current position 
[posnAbsolute, posnRelative] = motorGetPosition();
posnAbsolute(4:end) = []; %VI042611: Do not support save/load of position 4-vectors
posnRelative(4:end) = []; %VI042611: Do not support save/load of position 4-vectors

%Expand positionVectors array to be of implied size
if any([positionNumber positionNumber]> size(state.motor.positionVectors,1))
    for i=size(state.motor.positionVectors,1)+1:max(state.motor.position,positionNumber)
        [state.motor.positionVectors(i,:)] = nan;
    end
end

%Add new, or update, position vector
state.motor.positionVectors(positionNumber,:)=posnAbsolute; %Stored in absolute coordinates

%Display the newly stored position to command line -- using relative coordinates (matches what's displayed in Motor Controls GUI)
disp(['*** Motor Position #' num2str(positionNumber) ' defined as ' num2str(posnRelative(1)) ...
    ', ' num2str(posnRelative(2)) ', ' num2str(posnRelative(3)) ' ***']);

%If positionNumber was gotten from GUI-bound state var, then increment position number on GUI, to allow next position to be easily set from GUI
if nargin<1
    state.motor.position=state.motor.position+1;
    updateGUIByGlobal('state.motor.position');
end

		