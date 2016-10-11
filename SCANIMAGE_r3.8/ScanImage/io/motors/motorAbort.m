function motorAbort()
%MOTORABORT Abort the current move

global state
hMotor = state.motor.hMotor;
hMotorZ = state.motor.hMotorZ;

motorAbortFcn();
motorFinishMove();



function motorAbortFcn()
try
    hMotor.motorAbortFcn;
    hMotorZ.motorAbortFcn;
catch ME
    if ~strfind(ME.identifier,'InterruptMoveNotSupported')
        ME.rethrow();
    end
end

return;




% 
% function motorAbortFcn(hMot)
% try
%     if ~isempty(hMot)
%         hMot.moveInterrupt();
%     end
%     
% catch ME
%     if strfind(ME.identifier,'InterruptMoveNotSupported')
%         motorFinishMove(false);
%     else
%         ME.throwAsCaller();
%     end
% end
