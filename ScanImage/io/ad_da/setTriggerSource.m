function setTriggerSource(task,forceInternal)
%% function setTriggerSource(target,forceInternal)
% Updates the trigger source property of the specified Task
%
%% SYNTAX
%   task: A dabs.ni.daqmx.Task object to configure
%   forceInternal: Logical indicating whether to force internal triggering. If false, 'state.acq.externallyTriggered' determines internal vs. external triggering
%
%% NOTES
%   Rewritten from scratch to use new DAQmx adaptor -- Vijay Iyer 8/29/09
%
%% ****************************************************************************

global state

for i=1:length(task)
    if state.acq.externallyTriggered && ~forceInternal
        task(i).cfgDigEdgeStartTrig(state.init.externalTriggerInputTerminal);
    else
        task(i).cfgDigEdgeStartTrig(state.init.triggerInputTerminal);
    end
end
