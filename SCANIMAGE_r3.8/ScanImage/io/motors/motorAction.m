function varargout = motorAction(motorActionFcn,actionDescription, silentError)
%MOTORACTION Wrapper function for motor actions that handles case of motor errors in unified way
%
%% SYNTAX
%   motorActionFcn: Fucntion handle encapsulating motor-related action
%   actionDescription: (OPTIONAL) String description of action that, if provided, will be used in error message if error occurs
%   silent: (OPTIONAL) Logical indicating, if true, that error messaging will be silent (i.e. will not throw Matlab exception). Motor will simply be transitioned to error state. 
%
%% NOTES
%   This function draws from parts of MP285RobustAction and MP285Error, which are both now deprecated. -- Vijay Iyer 3/13/10
%
%   motorAction() throws any error during action as caller. This forces caller to abort, rather than continuing with its logic. 
%
%% CHANGES
%   VI120910A: Remove state.motor.movePending altogether -- Vijay Iyer 12/9/10
%
%% CREDITS
%   Created 3/13/10, by Vijay Iyer
%% **************************************************

global state

%Pass-through if motor feature is disabled
if ~state.motor.motorOn
    if nargout %If arguments are expected, better throw an error
        throwAsCaller(MException('ScanImage:MotorInactiveError', 'A motor action was attemped, but motor feature is disabled'));
    end
    return;
end

if nargin < 2 || isempty(actionDescription)
    actionDescription = '';
end

if nargin < 3 || isempty(silentError)
    silentError = false;
end

try  
    if state.motor.hMotor.errorCondition
        if ~silentError
            if ~isempty(actionDescription)
                fprintf(2,'WARNING: A pre-existing motor controller error condition prevented start of a motor operation (%s).\n',actionDescription);
            else
                fprintf(2,'WARNING: A pre-existing motor controller error condition prevented start of a motor operation.\n');
            end
        end
        return;
    end
    if nargout      
        varargout = cell(nargout,1);
        [varargout{:}] = feval(motorActionFcn); %Assumed that valid number of output arguments is used
    else
        feval(motorActionFcn);
	end
catch ME     
    %Create new, wrapped exception object, if needed
    %TODO: Consider using getReport() somehow, or displaying full error stack in other fashion
    if ~silentError || ~state.motor.hMotor.errorCondition
        if isempty(actionDescription);
            errString = sprintf('**********MOTOR ERROR*************\nError occurred during Motor control action:\n%s\n**********************************',ME.message);
        else            
            errString = sprintf('**********MOTOR ERROR*************\nError occurred during Motor control action (%s):\n%s\n**********************************',actionDescription,ME.message);
        end
        ME2 = MException('ScanImage:MotorError',errString);
        ME2.addCause(ME);
    end
    
    %Set motor object error condition, if it's not already been set (i.e. in course of executing motorActionFcn itself)
    if ~state.motor.hMotor.errorCondition                      
        state.motor.hMotor.errorConditionSet(ME2);
    end
    
    %Abort current acquisition, if one is in progress
    abortCurrent(true);  
        
    %state.motor.errorCond = 1;
    %state.motor.movePending=0; %VI120910A
    state.motor.lastPositionRead=[];
    
    %Reset motor position state/display
    state.motor.absXPosition = [];
    state.motor.absYPosition = [];
    state.motor.absZPosition = [];
    state.motor.absZZPosition = [];
    motorUpdatePositionDisplay(); %this updates position display to empty. relative origin is immaterial
    
    %Signal to GUI that error has occurred
    setStatusString('Motor Error!');
    turnOffMotorButtons;
    
    %Throw error, if indicated
    if ~silentError
        ME2.throwAsCaller();                              
    end    

end


