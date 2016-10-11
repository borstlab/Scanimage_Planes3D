function stopFocus(abort)
%% function stopFocus
%  Function that will stop the DAQ devices running for Focus 
% 
%% SYNTAX 
%   stopFocus()
%   stopFocus(abort)
%       abort: Logical value indicating, if true, that this is an 'abort' operation.
%
%% NOTES
%   Completely rewritten to use new DAQmx interface. To see prior version, see .MOLD file. -- Vijay Iyer 8/25/09
%
%   When 'abort' flag is used, the underlying DAQmx abort() method is used, instead of stop(), preventing error message -200010 from appearing.
%
%% CHANGES
%   VI031108A: Handle trigger timer, if present -- Vijay Iyer 3/11/08
%   VI071410A: Use state.init.hAcqTasks and state.init.hAOAcqTasks instead of determining them locally  -- Vijay Iyer 7/14/10
%   VI100610A: Stop exported clock Tasks, now separately from the 'acquisition' Tasks -- Vijay Iyer 10/6/10
%
%% CREDITS
%   Created 8/31/09, by Vijay Iyer
%   Based on earlier version by Tom Pologruto, 02/07/01
%% ************************************************************************

global state

state.internal.stopActionFunctions = 1; %VI032010A

if nargin < 1
    abort = false;
end

%%%VI071410A%%%
%get(state.init.eom.hAO,'writeTotalSampPerChanGenerated')

if abort
    state.init.hAcqTasks.abort();
    state.acq.hClockTasks.abort(); %VI100610A
else
    state.init.hAcqTasks.stop();
    state.acq.hClockTasks.stop(); %VI100610A
end
%%%%%%%%%%%%%%%


%%%VI071410A: Removed
% %Stop AI Task first
% if abort
%     state.init.hAI.abort();
% else
%     state.init.hAI.stop();
% end

% Handle trigger timer cleanup(VI031108A)
if ~isempty(state.internal.triggerTimer) 
    if strcmp(get(state.internal.triggerTimer,'Running'),'on')
        stop(state.internal.triggerTimer);
    end
    delete(state.internal.triggerTimer);
    state.internal.triggerTimer = [];
end  

%VI071410A: Unreserve buffer resources for AO acquisition Tasks
state.init.hAOAcqTasks.control('DAQmx_Val_Task_Unreserve');

%%%VI071410A: Removed
% %Stop Focus-related Tasks
% %tasks = state.init.hAO;
% aoTasks = state.init.hAO;
% if state.init.eom.pockelsOn
%     %tasks(end+1) = state.init.eom.hAO;
%     aoTasks(end+1) = state.init.eom.hAO;
% end
% 
% if abort
%     aoTasks.abort();
% else
%     aoTasks.stop();
% end
% aoTasks.control('DAQmx_Val_Task_Unreserve');





