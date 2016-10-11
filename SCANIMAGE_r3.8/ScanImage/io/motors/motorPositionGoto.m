function motorPositionGoto(positionID, statusStringWarning)
%MOTORPOSITIONGOTO Move motor to coordinates stored in particular motor position 
%
%% SYNTAX
%   positionID: Identifies the stored motor position number to which to move to. If empty/omitted, the current value of state.motor.position is used
%   statusStringWarning: (OPTIONAL) Logical indicating, if true, that warnings generated should be displayed to ScanImage status string, rather than command line. If omitted/empty, false is assumed.
%   
%% NOTES
%   Function replaces gotoPosition().
%
%% CHANGES
%   VI060610A: Use recently-added 'verify' option on motorSetPositionXXX() calls -- Vijay Iyer 6/6/10
%   VI051111A: Allow  NaN values to be present in position vectors
%   VI072111A: Handle case where no move is required -- Vijay Iyer 7/21/11
%
%% CREDITS
%   Created 3/26/10, by Vijay Iyer. Based on original function gotoPosition.
%% **************************

global state

if nargin<1 || isempty(positionID)
    positionID = state.hSI.selectedPositionID; %state.motor.position;
end

if nargin < 2 || isempty(statusStringWarning)
    statusStringWarning = false;
end

%Check if specified positionID has defined position
if ~state.hSI.positionDataStructure.isKey(positionID) %size(state.motor.positionVectors,1) < positionID 
    if statusStringWarning
        setStatusString(['Posn #' num2str(positionID) ' Undefined']);
    end
    fprintf(2,'WARNING (%s): Position #%d not defined.\n',mfilename, positionID);
    return
end

%%% VI051111A: Removed %%%%%%%
% %Check if coordinates stored at specified positionID is fully/correctly specified
% if all(isnan(state.motor.positionVectors(positionID,:)))
%     if statusStringWarning
%         setStatusString(['Posn #' num2str(positionID) ' Undefined']);
%     end
%     fprintf(2,'WARNING (%s): Position #%d not defined.\n',mfilename, positionID);
%     return
% end
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

positionStruct = state.hSI.positionDataStructure(positionID);

%VI072111A: Handle case where no move is required
if all(structfun(@isnan,positionStruct))
    return;
end

% TODO-DEQ20110825: this code is causing the new SI3 motor-read listener to fire twice--temporarily commenting it out, but review later.
% %Determine if length of move is allowed
% currPosn = motorAction(@motorGetPosition);
% if abs(positionStruct.motorX - currPosn(1))>state.motor.maxXYMove ... %VI032310A
%         || abs(positionStruct.motorY - currPosn(2))>state.motor.maxXYMove ... %VI032310A
%         || abs(positionStruct.motorZ - currPosn(3))>state.motor.maxZMove ... %VI032310A
%         || (state.motor.motorZEnable && abs(positionStruct.motorZZ - currPosn(4))>state.motor.maxZMove)
%     if statusStringWarning
%         setStatusString(['Position #' num2str(positionID) ' too far.']);
%     end
%     fprintf(2,['ERROR(%s): Position #%d is too far from current position. Cannot complete move operation.\n\t ' ...
%                   'To enables moves of this distance, change limits (state.motor.maxXYMove and state.motor.maxZMove) in internal.INI file\n'], mfilename);
%     return
% end

%Make the move
posnVector = [positionStruct.motorX positionStruct.motorY positionStruct.motorZ];
if state.motor.dimensionsXYZZ && ~state.hSI.posnIgnoreSecZ
    posnVector = [posnVector(:)' positionStruct.motorZZ];
end
%actionStr = ['Moving to Posn #' num2str(positionID)];
%setStatusString(actionStr); %VI032310B
motorSetPositionAbsolute(posnVector,'verify'); %VI060610A %VI032310B
setStatusString('');

