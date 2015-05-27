function acqDelayIncrement = computeAcqDelayIncrement(forceFine)
%Determines valid increment of acquisition delay parameter

global state

if nargin < 1
    forceFine = false;
end

%%%VI032409B%%%%%%%%%%%
if forceFine || state.internal.fineAcqDelayAdjust 
    acqDelayIncrement = 1/state.acq.inputRate * 1e6;    
else %Stick with integer values
    %%%VI032409B%%%%%%%%%%%%%%%
    factor = 1;
    while abs(round(1/state.acq.inputRate * 1e6 * factor) - (1/state.acq.inputRate * 1e6 * factor)) > 1e-3
        factor = factor+1;
        if factor > 10 %This shouldn't happen
            fprintf(2,'WARNING(%s): Acq Delay Increment computation behaved unexpectedly',mfilename);
            factor = 1;
            break;
        end
    end
    acqDelayIncrement = round(1/state.acq.inputRate * 1e6 * factor); %round shouldn't be needed, but req'd to ensure future rounds are correct
end
%%%%%%%%%%%%%%%%%%%%%%%

%%%VI032409B: Removed %%%%%%%%%%%%
% if state.acq.bidirectionalScan
%     acqDelayIncrement = 1/state.acq.inputRate * 1e6;   
% else    
%     if state.internal.increaseAORates
%         incrementOutputRate = state.acq.outputRate / state.internal.featureAORateMultiplier;
%     else
%         incrementOutputRate = state.acq.outputRate;
%     end
% 
%     acqDelayIncrement = 1/incrementOutputRate * 1e6;
% end   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%