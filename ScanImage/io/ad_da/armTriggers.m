function armTriggers(acqTasks,clockTasks,forceInternal,startEvent,nextEvent)
%% function armTriggers(acqTasks, forceInternal, startEvent, nextEvent)
% Prepares triggers, and related software events, according to configuration for upcoming acquisition
%
%% SYNTAX
%   acqTasks: Acquisition Tasks with start triggers to be configured for (if applicable)
%   clockTasks: Exported clock Tasks with start triggers to be configured for (if applicable)
%   forceInternal: Logical indicating whether to force internal start triggering. If false, 'state.acq.externallyTriggered' determines internal vs. external start triggering
%   startEvent: Logical indicating whether to arm generation of 'start' software event(s)
%   nextEvent: Logical indicating whether to arm generation of 'next' software events (provided a nextTrigger source is selected
%
%% NOTES
%   Arguments modulate the trigger/software-event configuration logic based on the type and state of acquisition (e.g. FOCUS/GRAB/LOOP, first time vs. succeeding times, etc)
%
%% CHANGES
%       VI100710A: Add special handling of exported clock Tasks
%
%% CREDITS
%   Created 9/19/09, by Vijay Iyer
%
%% ****************************************************************************
global state

%Parse input arguments
if nargin < 3 || isempty(forceInternal)
    forceInternal = false;
end

if nargin < 4 || isempty(startEvent)
    startEvent = false;
end

if nargin < 5 || isempty(nextEvent)
    nextEvent = false;
end

%Configures Start trigger for all Tasks, if any
if state.acq.externallyTriggered && ~forceInternal
    if state.acq.pureNextTriggerMode
        terminal = sprintf('PFI%d',state.acq.nextTrigInputTerminal);
        edge = state.acq.nextTrigEdge;
        startEvent = false; %Prevent duplicate dispatch of acquisitionStartedFcn() on first trigger -- it gets called via responding to the Next Event
    else
        terminal = sprintf('PFI%d',state.acq.startTrigInputTerminal);
        edge = state.acq.startTrigEdge;
    end
else
    terminal = state.init.triggerInputTerminal;
    edge = 'rising';
end

encodedEdge = trigEdgeEncode(edge);
for i=1:length(acqTasks)
    acqTasks(i).cfgDigEdgeStartTrig(terminal, encodedEdge);
end

%%%VI100710A: Handle Clock Tasks %%%%%%
for i=1:length(clockTasks)
    clockType = strrep(clockTasks(i).taskName,' ', '');
    clockType(1) = lower(clockType(1));
    
    %Arm start trigger, if not gated 
    if ~state.acq.clockExport.([clockType 'Gated'])
       clockTasks(i).cfgDigEdgeStartTrig(terminal, encodedEdge); 
    end    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Prepares Start Trigger 'sensor' (software event)
if startEvent
    state.init.hStartTrigCtr.set('sampClkSrc', terminal, 'sampClkActiveEdge', encodedEdge);
    state.init.hStartTrigCtr.start();
end

%Configure/arm Next trigger, if applicable
if ~isempty(state.acq.nextTrigInputTerminal) && nextEvent
    state.init.hNextTrigCtr.set('sampClkSrc', sprintf('PFI%d',state.acq.nextTrigInputTerminal), 'sampClkActiveEdge', trigEdgeEncode(state.acq.nextTrigEdge));
    state.init.hNextTrigCtr.start();
end

return;

%--------------------------------------------------------------------------
function argValue = trigEdgeEncode(riseOrFall)

switch lower(riseOrFall)
    case 'rising'
        argValue = 'DAQmx_Val_Rising';
    case 'falling'
        argValue = 'DAQmx_Val_Falling';
end

return;