function acqDelayOut = constrainAcqDelay(acqDelayIn,forceFine,roundFunc) 
%Computes constraint on acquisition delay to be a multiple of the increment value
global state 

%%%VI032409B%%%%
if forceFine || nargin < 3
    roundFunc = @round;
end
%%%%%%%%%%%%%%%%

acqDelayIncrement = computeAcqDelayIncrement(forceFine); %VI032409B
%Treat nearly integer values as integer-valued
if abs((acqDelayIn / acqDelayIncrement) - round(acqDelayIn / acqDelayIncrement)) < 1e-3
    roundFunc = @round;
end
acqDelayOut = roundFunc(acqDelayIn / acqDelayIncrement) * acqDelayIncrement; %VI032409B
