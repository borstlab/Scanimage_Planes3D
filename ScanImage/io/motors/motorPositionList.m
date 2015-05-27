function motorPositionList()
%MOTORPOSITIONGOTO Display to command line the list of stored positions, in relative coordinates
%
%% NOTES
%   Function replaces listPositions().
%
%% CHANGES
%   VI051111A: Specify correctly whether the coordinates displayed are relative or absolute -- Vijay Iyer 5/11/11
%
%% CREDITS
%   Created 3/26/10, by Vijay Iyer. Based on original function listPositions.
%% **************************

global state

relOrigin = motorGetRelativeOrigin();
useRelative = any(find(relOrigin)); %VI051111A
    
for pos=1:size(state.motor.positionVectors,1)

    if useRelative %VI05111A
        dispString = ['*** Stage Position #' num2str(pos) ' defined as relative position: ('];
    else
        dispString = ['*** Stage Position #' num2str(pos) ' defined as absolute position: ('];
    end
    disp([ dispString num2str(state.motor.positionVectors(pos,1)-relOrigin(1)) ', ' ...
            num2str(state.motor.positionVectors(pos,2)-relOrigin(2)) ', ' ...
            num2str(state.motor.positionVectors(pos,3)-relOrigin(3)) ') ***']);    
end

		

