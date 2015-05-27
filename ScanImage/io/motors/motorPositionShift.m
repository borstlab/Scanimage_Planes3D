function motorPositionShift(positionNumber,dimArray,statusStringWarning)
%MOTORPOSITIONSHIFT Shift motor position identified by positionNumber to current motor coordinates, and adjust all motor positions by this shift
%
%% SYNTAX
%   positionNumber: Identifies the stored motor position number to 'shift' to the current motor coordinates. This determines size of shift to apply to all motor positions. If empty/omitted, the current value of state.motor.position is used
%   dimArray: (OPTIONAL) Logical 1x3 array specifying which dimensions to determine and apply shift for. If empty/omitted, [1 1 1] is assumed. 
%   statusStringWarning: (OPTIONAL) Logical indicating, if true, that warnings generated should be displayed to ScanImage status string, rather than command line. If omitted/empty, false is assumed.
%   
%% NOTES
%   Function replaces applyShift and applyShiftXY. 
%
%% CREDITS
%   Created 3/26/10, by Vijay Iyer. Based on original functions applyShift() and applyShiftXY().
%% **************************

global state

if (nargin<1) || isempty(positionNumber)
    positionNumber=state.motor.position; %Get position number from GUI-bound state variable
end

if nargin < 2 || isempty(dimArray)
    dimArray = logical([1 1 1]);
else
    dimArray = logical(dimArray);
end

if nargin < 3 || isempty(statusStringWarning)
    statusStringWarning = false;
end

if isempty(state.motor.positionVectors)
    if statusStringWarning       
        setStatusString('No Positions Defined!');
    else
        fprintf(2,'WARNING (%s): No positions defined',mfilename);
    end    
    return
end

numPositions = size(state.motor.positionVectors,1);
if positionNumber > numPositions
    if statusStringWarning
        setStatusString(['Posn #' num2str(positionNumber) 'Undefined']);
    else
        fprintf(2,'WARNING (%s): Position #%d not defined.',mfilename, positionNumber);
    end
end

currPosn = motorGetPosition();
shift = currPosn - state.motor.positionVectors(positionNumber, :);
for i=1:numPositions
    state.motor.positionVectors(i,dimArray)=state.motor.positionVectors(i,dimArray) + shift(dimArray);
end


