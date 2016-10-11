function motorFinishMove()
% motorFinishMove Checks for the completion of a started move and/or that the final position is as intended
%
%% SYNTAX
%   out = motorFinishMove()
%       out: 1 if there's an error; 0 if successful
%
%% NOTES
%   Quick & dirty replacement for MP285FinishMove(), now that general LinearStageControllerBasic interface is used. This initial version maintains original logic as much as possible. -- Vijay Iyer 3/12/10
%
%   VI 4/26/2011: Should ensure that verifySetPosition() is safe in case that there was actually no move! (may occur, say, for one of the two motor controllers)
%
%% CHANGES
% 	Modified 2/5/1 by Bernardo Sabatini to support global state and preset serialPortHandle
%   VI050508A: Use MP285ReadAnswer() for more robust performance --  Vijay Iyer 5/5/2008 
%   VI093008A: Improve robustness/checking code -- Vijay Iyer 9/30/08
%   VI100608A: Employ MP285Interrupt() and updateMotorPosition() when respective error conditions are encountered-- Vijay Iyer 10/06/08
%   VI100808A: Attempt to refresh screen at end of move -- Vijay Iyer 10/08/08
%   VI101508A: Allow this function to be used when move has been interrupted, only to check the position -- Vijay Iyer 10/15/08
%   VI101508B: Revert VI100608A
%   VI120810A: Only set state.motor.movePending=false AFTER the moveWaitForFinish() has completed. -- Vijay Iyer 12/8/10
%   VI120910A: Remove state.motor.movePending altogether -- Vijay Iyer 12/9/10
%	VI102711A: Removed verifySetPosition() call, as this method is no longer supported by LSC/StageController classes
%
%% CREDITS
%   Created 3/12/10 by Vijay Iyer.  Based on MP285FinishMove created originally by Karel Svoboda, 8/28/00.
%
%% **************************

global state

if ~state.motor.motorOn
    return;
end

hMotor = state.motor.hMotor;
hMotorZ = state.motor.hMotorZ;

%%%VI120910A: Removed
% %Abort if no move is pending
% if ~state.motor.movePending
%     fprintf(2,'WARNING (%s): Called with no move pending and nothing to check.\n',mfilename);
%     return;
% end

%Check if move is finished
hMotor.moveWaitForFinish();

if state.motor.motorZOn
    hMotorZ.moveWaitForFinish();
end

%%%VI120910A: Removed
% %No longer flag the move as pending ...it's either over, or an error will be thrown
% state.motor.movePending = false; %VI120810A

%%%VI102711A: Removed%%%%%%
% %Check final position, if indicated
% if nargin < 1
%     checkPosn = 1;
% end
% 
% if checkPosn 
%     hMotor.verifySetPosition();
%     
%     if state.motor.motorZEnable
%         hMotorZ.verifySetPosition();
%     end
% end
