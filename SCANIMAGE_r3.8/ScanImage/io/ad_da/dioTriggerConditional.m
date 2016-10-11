function dioTriggerConditional
%% function dioTriggerConditional
%   Function issues a dio trigger /if/ ScanImage is configured for internal triggering. Otherwise, an external trigger is waited upon.
%   This feature is only available with DAQmx driver
%% NOTES
%
%% CHANGES
%   VI022808A Vijay Iyer 2/28/08 -- Added wait notification/timeout for case of waiting for external trigger
%   VI041308A Vijay Iyer 4/13/08 -- Avoid multiple timer constructions
%   VI110408A Vijay Iyer 11/04/08 -- Employ timer edit control to display amount of time spent waiting for an external trigger
%   VI092009A Vijay Iyer 9/20/09 - Eliminate use of whichNIDriver
%   VI122909A Vijay Iyer 12/29/09 -- Restore use of state.init.externalTriggerTimeout (an INI file variable)
%   VI092610A Vijay Iyer 9/26/10 -- state.init.externalTriggerTimeout is now state.acq.externalTriggerTimeout
%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global state

if ~state.acq.externallyTriggered %VI092009A
    state.internal.triggerTimer = [];
    dioTrigger;  
else
    if isempty(state.internal.triggerTimer) %VI041308A -- only create timer object if not done so before
        state.internal.triggerTimer = timer('Period',1.0,'StartDelay',0.5,'TimerFcn',@timerCallback,'TasksToExecute',state.acq.externalTriggerTimeout,'ExecutionMode','FixedRate');
    end
    start(state.internal.triggerTimer);
end

    function timerCallback(hObject,eventdata)
        %setStatusString(['Awaiting trigger...' num2str(get(hObject,'TasksExecuted')-1) 's']);
        setStatusString('Awaiting trigger...');
        state.internal.secondsCounter = get(hObject,'TasksExecuted'); %VI110408A
        updateGUIByGlobal('state.internal.secondsCounter'); %VI110408A
        
        if get(hObject,'TasksExecuted')==get(hObject,'TasksToExecute') %Handle timeout case
            abortCurrent;    
            setStatusString('No trigger received.');
        end    
    end

end
    
    
    
    
    

