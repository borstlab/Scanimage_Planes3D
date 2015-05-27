%% function setupAOObjects_Common
% Sets up Mirror Analog Output, and other Digital Output, Tasks with configuration independent settings
%
%% NOTES
%   Completely rewritten to use new DAQmx interface -- Vijay Iyer 8/25/09
%
%   At present, it is presumed that the Mirror and Pockels AO lines are on separate boards, and can thus be separate Tasks. 
%   Keeping them as separate Tasks allows them to potentially be run at different rates.
%   If a future NI board allows them to be run on the same board, then it won't be possible to keep them as separate Tasks. 
%
%% CHANGES
%   VI041209A: Compute minimum AO period increment here, eliminating unnecessarily duplicative INI file variable -- Vijay Iyer 4/12/09
%   VI090509A: Change to using timebase, rather than clock, synchronization -- Vijay Iyer 9/5/09
%   VI122309A: Create an AO Park task for each Pockels beam, so they can be individually set if/when required -- Vijay Iyer 12/23/09
%   VI091910A: Use newly named state.init.X/YMirrorChannelID -- Vijay Iyer 9/19/10
%
%% CREDITS
%   Created 8/25/09, by Vijay Iyer
%   Based on earlier version by Bernardo Sabatini/Tom Pologruto
%% ****************************************
function setupAOObjects_Common
global state
import dabs.ni.daqmx.*

%%%VI041209A: Compute minimum AO period increment here
state.internal.minAOPeriodIncrement = 1/state.internal.baseOutputRate; %VI041209A

%Create AO Tasks for Mirror lines and add channels
state.init.hAO = Task('Scan Mirror Output - GRAB');
%state.init.hAOF = Task('Scan Mirror Output - FOCUS');
state.init.hAOPark = Task('Scan Mirror Output - Park');

%Create channels for X/Y mirrors, for all Mirror AO Tasks, and configure timing/triggering/channels for primary Tasks
mirrorTasks = [state.init.hAO state.init.hAOPark];
primaryMirrorTasks = [state.init.hAO];
for i=1:length(mirrorTasks)
    xChan = mirrorTasks(i).createAOVoltageChan(state.init.mirrorOutputBoardID, state.init.XMirrorChannelID, 'X-Mirror',-state.init.outputVoltageRange,state.init.outputVoltageRange); %VI091910A
    yChan = mirrorTasks(i).createAOVoltageChan(state.init.mirrorOutputBoardID, state.init.YMirrorChannelID, 'Y-Mirror',-state.init.outputVoltageRange,state.init.outputVoltageRange); %VI091910A
    
    %Configure GRAB and FOCUS AO Mirror Tasks. The PARK AO Mirror Task does not require configuration (default timing/triggering is OK).
    if ismember(mirrorTasks(i),primaryMirrorTasks)
        mirrorTasks(i).cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps'); %TODO: Revisit whether final argument is req'd here
        mirrorTasks(i).cfgDigEdgeStartTrig(state.init.triggerInputTerminal);
        %set([xChan yChan], 'useOnlyOnBrdMem',true); 
    end
end


%Create/configure Shutter DO Task, if needed
if state.shutter.shutterOn 
    %%%Create new DO Task for shutter only if a separate board is used
    state.shutter.hDO = Task('Shutter Digital Output');
    state.shutter.hDO.createDOChan(state.shutter.shutterBoardID, ['line' num2str(state.shutter.shutterLineIndex)]);
    closeShutter;
end


%Handle Pockels boards/channels, if needed
if state.init.eom.pockelsOn        
    %Create/Configure Pockels AO tasks
    state.init.eom.hAO = Task('Pockels Output');  
    %state.init.eom.hAOPark = Task('Pockels Output - Park'); %Task used for On-Demand single sample outputs. No clock, no trigger. %VI122309A
    for i=1:state.init.eom.numberOfBeams
        pockelsBoardID = state.init.eom.(['pockelsBoardID' num2str(i)]);
        pockelsChanID = state.init.eom.(['pockelsChannelIndex' num2str(i)]);
        
        state.init.eom.hAO.createAOVoltageChan(pockelsBoardID,pockelsChanID,['PockelsCell-' num2str(i)],-10, 10);
        
        state.init.eom.(['hAOPark' num2str(i)]) = Task(['Pockels Output - Park Beam ' num2str(i)]); %VI122309A
        state.init.eom.(['hAOPark' num2str(i)]).createAOVoltageChan(pockelsBoardID,pockelsChanID,['PockelsCell-' num2str(i)],-10, 10); %VI122309A        
     
    end
    
    %state.init.eom.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps',[], state.init.outputBoardClockTerminal); %VI090509A %Set sample clock source to that exported from Mirror board AO subsystem
    state.init.eom.hAO.cfgSampClkTiming(state.acq.outputRate, 'DAQmx_Val_FiniteSamps'); %VI090509A: Use timebase synchronization instead
    state.init.eom.hAO.cfgDigEdgeStartTrig(state.init.triggerInputTerminal);
    
    %set(state.init.hAOPockels,'useOnlyOnBrdMem',true);    
end
    
    
    
    
    
