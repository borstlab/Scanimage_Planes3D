function setupAIObjects_Common
%% function setupAIObjects_Common
% Sets up components of AI Tasks that are config independent
%
%% NOTES
%   Completely rewritten to use new DAQmx interface -- Vijay Iyer 8/25/09
%   
%   The possibility that photodiode inputs are on same board as PMT inputs is handled, but not case that Zoom input is on same board. Will address if there is an actual use case for Zoom input. -- Vijay Iyer 8/25/09
%   The semi-arbitary AIZoom settings are chosen to match that of original version of this function.
%
%% CHANGES
%   VI091309A: Store photodiode channel conflicts into state.init.primaryBoardPhotodiodeChans INI variable -- Vijay Iyer 9/13/09
%   VI122309A: Register DoneEvent callback for hAI Task with error handler; this event should only get invoked by an error, since Task always uses DAQmx_Val_ContSamps. -- Vijay Iyer 12/23/09
%   VI031110A: Handle cases where max number of PMT channels is restricted by sample rate limitations (e.g. M series)
%   VI071510A: BUGFIX - Ensure the state.init.hAIPhotodiode array has an element, empty if needed, for every beam -- Vijay Iyer 7/15/10
%   VI090710A: Use the new auto-read capability of everyNSamples event -- Vijay Iyer 9/7/10
%   VI091010A: BUGFIX - Continuation of VI071510A, making sure it takes effect -- Vijay Iyer 9/10/10
%   VI120810A: Override state.init.maximumNumberOfChannels here when board has smaller number of channels available -- Vijay Iyer 12/8/10
%   VI082911A: Per Valentin Stein, insert changes required for proper operation when acquisition board is PXI-based
%
%% CREDITS
%   Created 8/25/09, by Vijay Iyer
%   Based on earlier version by Bernardo Sabatini/Tom Pologruto
%% *******************************************************

global state 
import dabs.ni.daqmx.*

hDevice = Device(state.init.acquisitionBoardID);

%VI082911A: Handle PXI case
if state.internal.usingPXI
    hDevice.apiCall('DAQmxResetDevice', state.init.acquisitionBoardID); %reset Device VS+NK 
end

%Determine max number of AI channels supported by specified on primary AI device

physChanList = get(hDevice,'AIPhysicalChans');
if isempty(physChanList)
    boardNumChans = 0;
else
    boardNumChans = length(strfind(physChanList,',')) + 1;  %physChanList is a comma-separated list
end
if ~get(hDevice,'AISimultaneousSamplingSupported') 
    maxAIRate = get(hDevice,'AIMaxSingleChanRate'); %VI110110A
    
    if get(hDevice,'AIMaxMultiChanRate')/state.init.maximumNumberOfInputChannels < state.internal.baseInputRate
        boardNumChans = double(maxAIRate >= state.internal.baseInputRate); %
    end
else %VI110110A
   maxAIRate = get(hDevice,'AIMaxMultiChanRate');
end

%VI120810A: Must override the state.init.maximumNumberOfInputChannels, as this is used widely throughout ScanImage code
state.init.maximumNumberOfInputChannels = min(state.init.maximumNumberOfInputChannels,boardNumChans);

%Determine maximum rate to allow, relative to our base rate (1.25MHz)
state.internal.maxInputRateMultiplier = min(4,floor(maxAIRate/state.internal.baseInputRate)); %VI110110A

%Determine channels on primary AI device that have been co-opted for photodiode inputs
photodiodeChanActive = zeros(state.init.eom.maxNumberOfBeams,1,'uint8');
if state.init.eom.pockelsOn
    for i=1:state.init.eom.maxNumberOfBeams
        [boardIDField, chanIDField] = getPhotodiodeFields(i);
        if isfield(state.init.eom,boardIDField)
            boardID = state.init.eom.(boardIDField);
            if ~isempty(boardID)
                if isfield(state.init.eom,chanIDField) && ~isempty(state.init.eom.(chanIDField))
                    photodiodeChanActive(i) = true;
                    if strcmpi(boardID, state.init.acquisitionBoardID)
                        state.init.primaryBoardPhotodiodeChans = [state.init.primaryBoardPhotodiodeChans state.init.eom.(chanIDField)]; %VI091309A
                    end
                else
                    error(['The field ''state.init.eom.' chanIDField ''' must be specified in the INI file, if the ''state.init.eom.' boardIDField ''' is non-empty.']);
                end              
            end
        end
    end
end

%Create PMT AI Tasks for GRAB/LOOP, FOCUS, PMT Offset, and Zoom operations, respectively
state.init.hAI = Task('PMT Inputs');
if state.init.autoReadZoom
    state.init.hAIZoom = Task('Zoom Inputs');
end

%Add all available channels to PMT AI Task, except any used for Photodiode input
for j=1:state.init.maximumNumberOfInputChannels %VI031110A: Was 'max', now 'min'. Never matters in practice. % DEQ20110125 - 'min' logic moved to line 47
    if ~ismember(j-1,state.init.primaryBoardPhotodiodeChans)
        voltageRange = state.acq.(['inputVoltageRange' num2str(j)]);
        hChan = state.init.hAI.createAIVoltageChan(state.init.acquisitionBoardID,j-1,['Imaging-' num2str(j-1)],-voltageRange,voltageRange); %#ok<SETNU>
        eval(['state.init.inputChannel' num2str(j) ' = hChan;']);
    end
end

%Configure Timing/Triggering/Channels/Callback for PMT AI Task
state.init.hAI.cfgSampClkTiming(state.acq.inputRate,'DAQmx_Val_ContSamps'); %Defer buffer configuration until everyNSamples value is known
state.init.hAI.cfgDigEdgeStartTrig(state.init.triggerInputTerminal,'DAQmx_Val_Rising');
state.init.hAI.registerDoneEvent(@DataMissedFcn); %VI122309A
state.init.hAI.everyNSamplesReadDataEnable=true; %VI090710A
state.init.hAI.everyNSamplesReadDataTypeOption = 'native'; %VI090710A

%VI082911A: Handle PXI case
if state.internal.usingPXI
    state.init.hAI.start(); %added to start communication with channels NK+VS
    state.init.hAI.stop();
end

%Store AI device information
state.acq.inputBitDepth = state.init.hAI.channels(1).get('resolution'); %For some reason, this is a Channel, rather than Device property. Anyway, should be same for all the channels on all the PMT AI tasks

%Handle AI Zoom Task -- this follows logic of original ScanImage implementation
if state.init.autoReadZoom
    state.init.hAIZoom = Task('Zoom Inputs');
    state.init.hAIZoom.createAIVoltageChan(state.init.zoomBoardID, state.init.zoomChannelIndex,'Zoom',-10,10)
    state.init.hAIZoom.cfgSampClkTiming(10000,'DAQmx_Val_FiniteSamps',100); %10ms acq
    state.init.hAIZoom.registerEveryNSamplesEvent(@calculateZoom,100);  
end

%Handle AI Photodiode Task creation/configuration, if Pockels feature is enabled
if state.init.eom.pockelsOn % && any(photodiodeChanActive) %VI091010A
    state.init.hAIPhotodiode = cell(state.init.eom.numberOfBeams,1); %VI071510A
    for i=1:state.init.eom.maxNumberOfBeams
        if photodiodeChanActive(i)
            state.init.hAIPhotodiode{i} = Task(['Photodiode Input ' num2str(i)]);
            [boardID, chanID] = getPhotodiodeFields(i);
            state.init.hAIPhotodiode{i}.createAIVoltageChan(state.init.eom.(boardID), state.init.eom.(chanID), ['Photodiode-' num2str(i)],-10,10);
            
            %Initialize sample clock timing and triggering, since they never change
            state.init.hAIPhotodiode{i}.cfgSampClkTiming(state.internal.eom.calibrationSampleRate, 'DAQmx_Val_FiniteSamps',2); %Acquisition (buffer) size will be set differently in different Task use contexts
            state.init.hAIPhotodiode{i}.cfgDigEdgeStartTrig(state.init.triggerInputTerminal);
        end
    end
    
end

   
    function [boardIDField, chanIDField] = getPhotodiodeFields(beamNumber)
        boardIDField = ['photodiodeInputBoardID' num2str(beamNumber)];
        chanIDField = ['photodiodeInputChannel' num2str(beamNumber)];
    end

end

