function motorRecover()
%% function motorRecover()
%   Recover from Motor error condition
%
%% NOTES
%   TODO: Put Recover button in a disabled state while any pending action is being tried, then restore if recover/reset/done attempt failed (just call turnOffMotorButtons) -- Vijay Iyer 4/5/10
%   TODO: Consider  adding errorCondition/Suspend() to DClass definition, avoiding need to manually cache error condition messages while trying to clear it
%
%% CHANGES
%   VI051211A: Support secondary Z motor controller cases -- Vijay Iyer 5/11/11
%
%% CREDITS
%   Created 4/2/10, by Vijay Iyer
%% ************************************************

global state

%Pass-through if motor feature is disabled
if ~state.motor.motorOn
    return;
end

hMotor = state.motor.hMotor;
hMotorZ = state.motor.hMotorZ;

motorRecoverHidden(hMotor,state.motor.safeReset);
if ~isempty(hMotorZ)
    motorRecoverHidden(hMotorZ,state.motor.safeResetZ);
end

motorGetPosition();

turnOnMotorButtons();
setStatusString('Motor Restored');

return;

function motorRecoverHidden(hMotor,safeReset)

% %Cache original error message
% if hMotor.errorCondition
%     cachedException = hMotor.errorConditionArray(end); %Only store last, as there's presently no way to set more than one
% else
%     return;
%     %error('The operation motorRecover() was invoked although no Motor error condition is present');
% end

try
    try
        hMotor.recover();
    catch ME        
        if safeReset 
            try
                hMotor.reset();
            catch %#ok<CTCH>
                promptHardReset(hMotor);
            end
        else
            promptHardReset(hMotor);
        end
    end
catch ME
    ME.throwAsCaller();
end

return;


function promptHardReset(hMotor)
resp = questdlg({sprintf('Software reset of Motor Controller (%s) failed or is not possible.', class(hMotor)) 'Physically reset or power-cycle the motor controller, and then hit Done.' '(Or hit Cancel to abort Motor Recover)'},'Hard Motor Reset Required','Done','Cancel','Done');

switch resp
    case 'Cancel'
        return;
    case 'Done'
        hMotor.recover();
end

return;


% function restoreMotorOperation(hMotor,trapError)
% try
%     hMotor.recover();
%     hMotor.positionAbsolute; %This will cause error if attempt failed
% catch ME2
%     if nargin >= 2 && trapError
%         hMotor.lscSetErrorFlag(); 
%         setStatusString('Recover Failed!')
%         fprintf(2,'WARNING: Attempt to recover from Motor error failed (controller of class ''%s'')!\n',class(hMotor));
%         return;
%     else
%         ME2.rethrow();
%     end
% end
% 
% return;

