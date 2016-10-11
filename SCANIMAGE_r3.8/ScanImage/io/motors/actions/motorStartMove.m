%% function motorStartMove(newPosn,coordType)
%  Start a motor move in nonblocking fashion. Move completion can be later confirmed/awaited using motorFinishMove(). 
%
%% SYNTAX
%   newPosn: <OPTIONAL> Position, given as 1,3,or 4 element array, to which to start a move operation.
%               If omitted/empty, the current state.motor.abs<X,Y,Z,ZZ>Position or state.motor.rel<X,Y,Z,ZZ>Position value will be used. 
%               If one element array provided, then move is presumed to be Z only (ZZ if state.motor.dimensionsXYZZ & state.motor.motorZEnable are both true)%
%
%   coordType: <OPTIONAL; default='absolute'> One of {absolute,relative}, specifying the type of position specified by newPos, or which position state variable to use.
%   startBothMotors: <OPTIONAL; default=false> If true, when specified
%
%% NOTES
%   Rewritten from scratch, and renamed from startMoveStackFocus(), as part of refactoring to use new LinearStageControllerBasic class. See MOLD file (startMoveStackFocus.mold) for original version. -- Vijay Iyer 3/13/10   
%   
%   Original version was focused only on stack operations. Have now changed so this is a general function tied to moveStartXXX() operations of LinearStageController class -- Vijay Iyer 6/6/10
%
%   Depending on state.motor.motorEnable & state.motor.dimensionsXYZZ, move may involve either 1 or 2 scanimage.StageController objects
%
%   TODO: Consider whether there is ever any good reason to interrupt move when one is found pending (this original logic was left intact here for now) -- Vijay Iyer 3/13/10   
%
%% CHANGES
%   VI101608A: Compute X/Y/Z positions from initial position, rather than assuming that current values for AbsX/Y/ZPosition is correct. This allows state to be restored if absolute position has been corrupted. -- Vijay Iyer 10/16/18
%   VI052909A: Determine Power vs Z feature power scaling factor for each beam here -- Vijay Iyer 5/29/09
%   VI010610A: Use state.init.eom.powerLzArray in all cases now - if override is in effect, then set already in acquisitionStartedFcn() now -- Vijay Iyer 1/6/10
%   VI032010A: Use motorCheckPendingMove() to check/handle pending move, if any. -- Vijay Iyer 3/20/10
%   VI060610A: Relocated stack-specific aspects of this function to makeFrameByStripes\motorStackStartMove; added ability to specify position as argument; this function is now more general -- Vijay Iyer 6/6/10
%   VI120910A: Remove state.motor.movePending altogether -- Vijay Iyer 12/9/10
%   VI051111A: Support secondary Z motor use cases -- Vijay Iyer 5/11/11
%
%% CREDITS
%   Rewritten from scratch 3/13/10, by Vijay Iyer. Original version by Tom Pologruto, 01/05/2001.
%
%% ************************
function motorStartMove(newPosn,coordType)

global state

if ~state.motor.motorOn
    return;
end

hMotor = state.motor.hMotor;
hMotorZ = state.motor.hMotorZ;

%Not sure this should ever happen
if motorErrorPending()
    fprintf(2,'WARNING (%s): Existing MP-285 error prevented attempt to start stack movement\n',mfilename);
    return;
end

%Check if move is pending and try interrupt, if so. 
%motorCheckPendingMove(); %VI120910A %VI032010A

%VI060610A: Determine position to move to from arguments and state variables (if needed)
if nargin < 1 || isempty(newPosn)
    newPosn = [state.motor.absXPosition state.motor.absYPosition state.motor.absZPosition state.motor.absZZPosition];
    coordType = 'absolute';
elseif nargin < 2 || isempty(coordType)
    coordType= 'absolute';
end
   

%%%VI051111A: Start the move %%%
switch lower(coordType)
    case 'absolute'
        moveStartFcn = @hMotor.moveStartAbsolute;
        moveStartFcnZ = @hMotorZ.moveStartAbsolute;
    case 'relative'
        moveStartFcn = @hMotor.moveStartRelative;
        moveStartFcnZ = @hMotorZ.moveStartRelative;
    otherwise
        error('The ''coordType'' value must be either ''absolute'' or ''relative''');
end


    if isscalar(newPosn) %Only move one motor, in its Z dimension only
        if state.motor.motorZEnable
            moveStartFcnZ([nan nan newPosn]);
        else
            moveStartFcn([nan nan newPosn]);
        end        
    elseif state.motor.dimensionsXYZZ %XYZ-Z: move 1 or 2 motors to specified 3 or 4-vector coordinates, regardless of motorZEnable state
        moveStartFcn(newPosn(1:3));
        if length(newPosn) > 3
            moveStartFcnZ([nan nan newPosn(4)]);
        end
    elseif state.motor.motorZEnable %XY-Z: move both motors to specified 3-vector coordinates
        moveStartFcn([newPosn(1:2) nan]);
        moveStartFcnZ([nan nan newPosn(3)]);        
    else %Move primary motor to specified 3-vector coordinates (whether XYZ, XY, or Z)
        moveStartFcn(newPosn(1:3));
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%VI120910A: Removed
%state.motor.movePending = 1; %Flag that move has started


