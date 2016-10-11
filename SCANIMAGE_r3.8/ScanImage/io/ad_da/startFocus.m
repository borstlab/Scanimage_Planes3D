function startFocus
%% function startFocus
% Starts the DAQ objects for focusing (hAO, hAIF).
%
%% NOTES
%   Function rewritten completely from scratch. Previous version can be seen in .MOLD file. -- Vijay Iyer 8/28/09
%
%   Pockels data must be computed in this function, since the 'flybackOnly' flag of makePockelsCellDataOutput() is used in FOCUS mode, but not in GRAB/LOOP modes -- Vijay Iyer 9/5/09 
%
%   This function illustrates DAQmx point that either CfgSampClktiming (used for Mirror AO), or setting the appropriate properties directly (used for Pockels AO), can be used to selectively update Task's timing properties. 
%
%% CHANGES
%   4/13/08 Vijay Iyer (VI041308A): Force trigger source to be internal before starting channels
%   9/19/09 Vijay Iyer (VI091909A): Use (new) armTrigger() and start() on all acquisition tasks en masse
%   7/14/10 Vijay Iyer (VI071410A): Use state.init.hAcqTasks instead of constructing local acqTaskList variable
%   VI092310A: Update the new lastStartMode and freshData flag variables -- Vijay Iyer 9/23/10
%   VI100610A: Arm triggers and start exported clock Tasks, now separately from the 'acquisition' Tasks -- Vijay Iyer 10/6/10
%   VI111810A: Avoid re-registering makeStripe() callback during stopAndRestartFocus() actions; this had been leading to seg violations with DAQmxExportSignal always at top of stack -- Vijay Iyer 11/18/10
%
%% CREDITS
%   Created 8/28/09, by Vijay Iyer
%   Based heavily on earlier version by Tom Pologruto, CSHL, February 2000
%
%% ***********************************************************************

global state

%acqTaskList = [state.init.hAI state.init.hAO]; %VI071410A %VI092009A

if state.init.eom.pockelsOn == 1       
   
    %Write output data to Pockels AO Task
    state.init.eom.hAO.writeAnalogData(makePockelsCellDataOutput(state.init.eom.focusLaserList),1);  %Specify that this is 'flyback only'
    
    %Configure Pockels AO timing
    if state.acq.infiniteFocus %VI022108A
        state.init.eom.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_ContSamps');
    else
        state.init.eom.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps',state.init.eom.hAO.get('bufOutputBufSize') * state.internal.numberOfFocusFrames);
    end
    
    %%%VI092009A%%%%%%%%
    %     %Configure Pockels AO triggering
    %     setTriggerSource(state.init.eom.hAO,true); %VI041308A
    %
    %     %Start Pockels AO Task
    %     start(state.init.eom.hAO);
    %%%%%%%%%%%%%%%%%%%%
    
    %acqTaskList = [acqTaskList state.init.eom.hAO]; %VI071410: Removed 
end


if state.acq.infiniteFocus
    state.init.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_ContSamps');
else
    state.init.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps',state.init.hAO.get('bufOutputBufSize') * state.internal.numberOfFocusFrames);
end


%state.init.hAI.registerEveryNSamplesEvent(); %Unregisters previous callback
if ~state.internal.fastRestart %VI111810A
    state.init.hAI.everyNSamplesEventCallbacks = state.hSI.hMakeStripe;
end

%setTriggerSource([state.init.hAI state.init.hAO],true); %VI092009A: Removed %VI041308A 
%start([state.init.hAI state.init.hAO]); %VI092009A

state.internal.stopActionFunctions = 0; %VI032010A
armTriggers(state.init.hAcqTasks, state.acq.hClockTasks,true,false,false); %VI100610A %VI071410A %VI092009A
if state.acq.infiniteFocus
    exportClocks(inf);%TO091210B - %TO091210B %This has to come after `armTriggers`, to potentially disable triggering for gated clocks.
else
    exportClocks(state.internal.numberOfFocusFrames);
end
start(state.init.hAcqTasks); %VI071410A %VI092009A

%%%VI100610A%%%
if state.acq.clockExport.exportOnFocus
    start(state.acq.hClockTasks);
end
%%%%%%%%%%%%%%%

state.internal.lastStartMode = 'focus'; %VI092310A
state.internal.freshData = true; %VI092310A
