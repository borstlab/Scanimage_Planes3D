function startSnapShot
%% function startSnapShot
% Function that will start the DAQ devices running for SnapShot (hAO, hAI)
%
%% NOTES
%   Function rewritten completely from scratch. Previous version can be seen in .MOLD file. -- Vijay Iyer 9/03/09
%
%% CHANGES
% VI041308A - Set the trigger source to internal before starting channels -- Vijay Iyer 4/13/2008
% VI041308B - Pull out the cell array creation from the snapLaserList var -- Vijay Iyer 4/13/2008
% VI041808A - Use LUT value to determine minimum value
% VI011609A: Changed state.init.pockelsOn to state.init.eom.pockelsOn -- Vijay Iyer 1/16/09
% VI092110A: Configure timing /before/ writing analog data, for X series compatibility (NI Ref # 7297835) -- Vijay Iyer 9/21/10
% VI092110B: Use armTriggers() function and start acq tasks en masse, as done in startGrab()/startFocus() -- Vijay Iyer 9/21/10
% VI092310A: Update the new lastStartMode/freshData flag variables -- Vijay Iyer 9/23/10
% VI100610A: Actually generate the Pockels cell output on Snapshot, as apparently intended; leave out clock export signals -- Vijay Iyer 10/6/10
%
%% CREDITS
% Created 9/9/09, by Vijay Iyer
% Based on earlier version by Tim O'Connor, 4/23/04
%
%% *******************************************************

global state;


if state.init.eom.pockelsOn == 1  
    
    %Determine Pockels output data
    state.init.eom.hAO.writeAnalogData(makePockelsCellDataOutput(state.init.eom.snapLaserList)); %VI092110A    

    %Configure output buffering based on whether there is, or isn't, power modulation
    if state.init.eom.usePowerArray || any(state.init.eom.showBoxArray) || any(state.init.eom.uncagingMapper.enabled)
        state.init.eom.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps',state.init.eom.hAO.get('bufOutputBufSize')); %VI092110A %Buffer length equals length of acquisition...no need to repeat
    else
        state.init.eom.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps',state.init.eom.hAO.get('bufOutputBufSize') * state.acq.numberOfFrames); %VI092110A %Buffer length is of a single frame
    end
        
    
    %%%VI092110B: Removed%%%%%%%
    %     %Configure Pockels AO triggering
    %     setTriggerSource(state.init.eom.hAO,true); %VI041308A
    %
    %     %Start Pockels AO Task
    %     start(state.init.eom.hAO);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

state.init.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps',state.init.hAO.get('bufOutputBufSize') * state.acq.numberOfFrames);

%DEQ20101222state.init.hAI.everyNSamplesEventCallbacks = @makeFrameByStripes;
state.init.hAI.everyNSamplesEventCallbacks = state.hSI.hMakeFrameByStripes;

%setTriggerSource([state.init.hAO state.init.hAI],true); %VI092110B %Force internal triggering; don't use TriggerCtr, i.e. no acquisitionStartedFcn()
armTriggers(state.init.hAcqTasks, [], true, true, false); %VI092110B
%start([state.init.hAI state.init.hAO]);
start(state.init.hAcqTasks); %VI100610A

state.internal.lastStartMode = 'snap'; %VI092310A
state.internal.freshData = true; %VI092310A

