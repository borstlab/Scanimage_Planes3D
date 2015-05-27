function stopGrab(abort)
%% function stopGrab
%  Function that will stop the DAQ devices running during GRAB/LOOP Acquisitions 
% 
%% SYNTAX 
%   stopGrab()
%   stopGrab(abort)
%       abort: Logical value indicating, if true, that this is an 'abort' operation.
%
%% NOTES
%   Completely rewritten to use new DAQmx interface. To see prior version, see .MOLD file. -- Vijay Iyer 8/25/09
% 
%% CHANGES
%   3/11/08 Vijay Iyer (VI031108A) - Handle trigger timer cleanup, if needed
%   8/21/08 Vijay Iyer (VI082108A) - Close tifStream, if any
%   9/19/09 Vijay Iyer (VI091909A) - Use unarmTriggers() call to clean up any trigger 'sensor' counter-input Tasks
%   7/14/10 Vijay Iyer (VI071410A) - Use state.init.hAcqTasks and state.init.hAOAcqTasks instead of determining them locally  -- Vijay Iyer 7/14/10
%   VI092210A: Check state.files.autoSave instead of now defunct state.acq.saveDuringAcquisition -- Vijay Iyer 9/22/10
%   VI100610A: Stop exported clock Tasks, now separately from the 'acquisition' Tasks -- Vijay Iyer 10/6/10
% 
%% CREDITS
%   Created 9/01/09, by Vijay Iyer
%   Based on earlier version by Tom Pologruto, 02/07/01
%% **********************************************************

global state

state.internal.stopActionFunctions = 1; %VI032010A

if nargin < 1
    abort = false;
end

%%%VI071410A%%%
% samps1 = get(state.init.eom.hAO,'writeTotalSampPerChanGenerated');
% pause(0.1);
% samps2 = get(state.init.eom.hAO,'writeTotalSampPerChanGenerated');
%fprintf('EOM Samples Generated: %d @ Time 1, %d @ Time 2\n',samps1,samps2);
if abort
    state.init.hAcqTasks.abort();
    state.acq.hClockTasks.abort(); %VI100610A
else
    state.init.hAcqTasks.stop();
    state.acq.hClockTasks.stop(); %VI100610A
end
%%%%%%%%%%%%%%%

% Handle trigger timer cleanup(VI031108A)
if ~isempty(state.internal.triggerTimer) 
    if strcmp(get(state.internal.triggerTimer,'Running'),'on')
        stop(state.internal.triggerTimer);
    end
    delete(state.internal.triggerTimer);
    state.internal.triggerTimer = [];
end   

%%%VI071410A: Removed
% %Stop Grab-related Tasks
% tasks = [state.init.hAI state.init.hAO];
% aoTasks = [state.init.hAO];
% if state.init.eom.pockelsOn == 1 
%     tasks(end+1) = state.init.eom.hAO;
%     aoTasks(end+1) = state.init.eom.hAO;
% end
% 
% if abort
%     tasks.abort();
% else
%     tasks.stop();
% end
% aoTasks.control('DAQmx_Val_Task_Unreserve');

%VI071410A: Unreserve buffer resources for AO acquisition Tasks
state.init.hAOAcqTasks.control('DAQmx_Val_Task_Unreserve');

unarmTriggers(); %VI091909A

%VI082108A: Handle tifStream closing, if needed
if state.internal.abortActionFunctions && state.files.autoSave && ~isempty(state.files.tifStream) %VI092210A
    try        
        close(state.files.tifStream);
        state.files.tifStream = [];
    catch        
        delete(state.files.tifStream,'leaveFile');
        errordlg('Failed to close an open TIF stream. A file may be corrupted.');
        state.files.tifStream = [];
    end            
end

%VI010609A: Update EOM GUI at end of aborted GRAB (might have been a stack acquisition with P vs Z variation)
if abort
    if state.init.eom.pockelsOn
        updateMaxPowerDisplay(state.init.eom.beamMenu);
    end
end