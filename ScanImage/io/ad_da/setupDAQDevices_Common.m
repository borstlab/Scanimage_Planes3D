
function setupDAQDevices_Common
%% function setupDAQDevices_Common
% Sets up the portions of AO and AI objects that are configuration independent
%
%% NOTES
%   Completely rewritten to use new DAQmx interface -- Vijay Iyer 8/25/09
%
%% CHANGES
%   VI090509A: Export timebase, rather than sample clock, after all, as it avoids DAQmx bug with Pockels AO Task -- Vijay Iyer 9/5/09
%   VI091009A: Use sampClkTimebase property, rather than masterClkTimebase or refClkTimebase, to allow for error-free Task-based routing -- Vijay Iyer 9/10/09
%   VI091309A: Use sampClkTimebase property only for M/X series; go back to using masterClkTimebase for other device families (only been tested on AO series devices at moment) -- Vijay Iyer 9/13/09
%   VI091709A: Initialize the newly created TriggerGUI -- Vijay Iyer 9/17/09
%   VI091909A: Use CI Task with a 'Signal' event for the hTriggerCtr Task -- Vijay Iyer 9/19/09
%   VI091909B: Create CI Task for Next trigger, if applicable -- Vijay Iyer 9/19/09
%   VI010810A: BUGFIX -- Must now determine the number of beams before setting up AO Tasks to avoid error -- Vijay Iyer 1/8/10
%   VI071410A: Store the installed 'acquisition Tasks' and AO-specific acquisition Tasks, i.e. those which must all be started/stopped at start/stop of all standard ScanImage acquisition types -- Vijay Iyer 7/14/10
%   VI071410B: Improve setTimebaseSource() and refactor code that uses it -- Vijay Iyer 7/14/10
%   VI071410C: Setup frame clock output Task, ensuring that its board is synchronized to master timebase on acquisition board -- Vijay Iyer 7/14/10
%   VI071510A: Use 'CtrTimebaseSrc/Rate' properties for importing timebase to clock export Tasks -- this works across device families -- Vijay Iyer 7/15/10
%   VI090810A: Only add blank entry to next/startTrigger popup menus where/if needed -- Vijay Iyer 9/8/10
%   VI090810B: Hide the EXT button if no external start trigger sources are available -- Vijay Iyer 9/8/10
%   TO091210B: Created exportClocks.m -- Tim O'Connor 9/12/10
%   VI100610A: Put exported clock initialization back into this function (replaces VI071410C & TO091210A) -- Vijay Iyer 10/6/10
%   VI100610B: Exported clock Tasks no longer part of state.init.hAcqTasks -- Vijay Iyer 10/6/10
%   VI112010A: Ensure both clock Board and Counter ID is specified before creating Task -- Vijay Iyer 11/20/10
%   VI112010B: Disable panel for frame/line/pixel clocks in Exported Clocks GUI when clock is not available from INI file -- Vijay Iyer 11/20/10
%   VI112010C: Don't automatically enable line clock gating by frame clock. Leave this as user option. Disabling this option allows frame & line clocks to have independent phase delays. -- Vijay Iyer 11/20/10
%   VI113010A: Set state.init.eom.numberOfBeams=0 when pockelsOn=0 -- helps to avoid EOM index errors -- Vijay Iyer 11/30/10
%   VI081211A: Use triggerBoardID, rather than mirrorBoardID, to specify the board for which Ctr0 serves as teh 'start trigger sensor' -- Vijay Iyer 8/12/11
%   
%% CREDITS
% Created 8/25/09, by Vijay Iyer
% Based on original version by Bernardo Sabatini/Tom Pologruto - February 2001
%% ******************************************
global state gh
import dabs.ni.daqmx.*

%% General stuff
state.init.hDAQmx = System();
%state.init.hPrimaryDevice = Device(state.init.mirrorOutputBoardID);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create Trigger Tasks
state.init.hTrigger = Task('Internal Start Trigger Source');
state.init.hTrigger.createDOChan(state.init.triggerBoardID, ['line' num2str(state.init.triggerLineID)]);
state.init.hTrigger.writeDigitalData(false, 0.2); %This places line in default state

%%%VI091909A%%%%
state.init.hStartTrigCtr = Task('Start Trigger Sensor'); %Task used to fire callback when start trigger occurrs
state.init.hStartTrigCtr.createCICountEdgesChan(state.init.triggerBoardID, state.init.startTrigCtr); %VI081211A
state.init.hStartTrigCtr.cfgSampClkTiming(1000, 'DAQmx_Val_HWTimedSinglePoint', [], state.init.triggerInputTerminal); %Sample rate is a 'dummy' value %NOTE: HwTimedSinglePoint doesn't work for USB M series devices, but is required for Counter In Event Counting Tasks -- Vijay Iyer 11/4/10
state.init.hStartTrigCtr.registerSignalEvent(@acquisitionStartedFcn, 'DAQmx_Val_SampleClock');
%%%%%%%%%%%%%%%

%%%VI091909B%%%%
if ~isempty(state.init.nextTrigBoardID) || ~isempty(state.init.nextTrigCtrID)
    if isempty(state.init.nextTrigCtrID) || isempty(state.init.nextTrigBoardID)
        fprintf(2,'WARNING: Both nextTrigBoardID and nextTrigCtrID must be set to enable next triggering feature. Feature disabled. \n');
    else
        state.init.hNextTrigCtr = Task('Stop Trigger Sensor'); %Task used to fire callback when stop/next trigger occurs
        state.init.hNextTrigCtr.createCICountEdgesChan(state.init.nextTrigBoardID, state.init.nextTrigCtrID);
        state.init.hNextTrigCtr.cfgSampClkTiming(1000, 'DAQmx_Val_HWTimedSinglePoint', [], 'PFI0'); %Sample rate is a 'dummy' values %NOTE: HwTimedSinglePoint doesn't work for USB M series devices, but is required for Counter In Event Counting Tasks-- Vijay Iyer 11/4/10
        state.init.hNextTrigCtr.registerSignalEvent(@nextTriggerFcn, 'DAQmx_Val_SampleClock');
    end
end
%%%%%%%%%%%%%%%%

%%%VI100610A: Exported Clock Tasks %%%%%%%%%%%
exportedClockTypes = {'Frame' 'Line' 'Pixel'};
for i=1:length(exportedClockTypes)
    
    clockType = exportedClockTypes{i};
    clockBoardID = state.init.([lower(clockType) 'ClockBoardID']);
    clockCtrID = state.init.([lower(clockType) 'ClockCtrID']); %VI112010A

    if ~isempty(clockBoardID) && ~isempty(clockCtrID) %VI112010A
        hCtr = Task([clockType ' Clock']);
        
        hCtr.createCOPulseChanTime(clockBoardID, clockCtrID, '', 1,1,0); %VI112010A %Dummy initial values for on/off times & initialDelay
        
        %Ensure counter is synchronized with primary board
        if ~strcmpi(clockBoardID, state.init.mirrorOutputBoardID)
            hCtr.channels.set('ctrTimebaseSrc', 'RTSI7', 'ctrTimebaseRate', 20e6);
        end
        
        %Identify if board supports finite CO generations
        hDev = Device(clockBoardID);
        state.init.([lower(clockType) 'ClockBoardSupportsFinite']) = ...
            ismember(hDev.productCategory, {'DAQmx_Val_XSeriesDAQ'});
        
        state.init.(['h' clockType 'ClkCtr']) = hCtr;
                
    else %VI112010B: Disable controls in panel of Exported Clocks GUI corresponding to non-enabled clock
        hPanel = gh.clockExportGUI.(['pnl' clockType 'Clock']);        
        set(findobj(hPanel,'-property','Enable'),'Enable','off'); %All non-panel children               
        set(findobj(hPanel,'Type','uipanel'),'ForegroundColor',[.6 .6 .6]); %All panel children        
    end
end

%Initialize clock 'gating' to reasonable defaults, where sensible. 
%'Gating' means <pause/start>-triggering for boards <without/with> finite counter output

%%%VI112010C: Removed%%%%
% if ~isempty(state.init.hFrameClkCtr) && ~isempty(state.init.hLineClkCtr) ...
%         && strcmpi(state.init.frameClockBoardID,state.init.lineClockBoardID)    
%     state.acq.clockExport.lineClockGatedEnable = 1;
%     updateGUIByGlobal('state.acq.clockExport.lineClockGatedEnable');
% end
%%%%%%%%%%%%%%%%%%%%%%%%%

if ~isempty(state.init.hLineClkCtr) && ~isempty(state.init.hPixelClkCtr) ...
        && strcmpi(state.init.lineClockBoardID,state.init.pixelClockBoardID)
    state.acq.clockExport.pixelClockGatedEnable = 1;
    updateGUIByGlobal('state.acq.clockExport.pixelClockGatedEnable');
end      

%Initialize list of active clock Tasks
state.acq.hClockTasks = Task.empty();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state.init.eom.numberOfBeams = 0; %VI113010A

%%%VI010810%%%%
if state.init.eom.pockelsOn
    %Determine number of 'beams' used -- this affects both number of Pockels AO and Photodiode AO channels/tasks.
    numPossiblePockels = state.init.eom.maxNumberOfBeams;
    numPockels = 0;
    %validPockels = [];
    
    for i = 1 : numPossiblePockels     
        fieldNames = {['pockelsBoardID' num2str(i)] ['pockelsChannelIndex' num2str(i)]};
        
        varsExist = cellfun(@(x)isfield(state.init.eom,x),fieldNames);
        
        if all(varsExist)            
            varsEmpty = cellfun(@(x)isempty(state.init.eom.(x)),fieldNames);
            
            if ~any(varsEmpty)              
                numPockels = numPockels + 1;
            else
                break;
            end
        else
            break;
        end
    end
        
    state.init.eom.numberOfBeams = numPockels;

    if numPockels == 0
        warning('No modulator (Pockels) beams have been correctly configured in INI file --> setting pockelsOn=false');
        state.init.eom.pockelsOn = false;
    end
end

%%%%%%%%%%%%%%%%%

%Determine if PXI boards are being used
hDevice = dabs.ni.daqmx.Device(state.init.acquisitionBoardID);
state.internal.usingPXI = ismember(hDevice.get('busType'),{'DAQmx_Val_PXI' 'DAQmx_Val_PXIe'});

%Create Common AO/AI Tasks
setupAOObjects_Common;
setupAIObjects_Common;

%%%VI071410A: Identify installed acquisition and AO acquisition Tasks
state.init.hAcqTasks = [state.init.hAI, state.init.hAO];
optionalAcqTasks = {state.init.eom.hAO};  %VI100610B
for i = 1 : length(optionalAcqTasks)
    if ~isempty(optionalAcqTasks{i})
        state.init.hAcqTasks(end + 1) = optionalAcqTasks{i};
    end
end

state.init.hAOAcqTasks = state.init.hAO;
if state.init.eom.pockelsOn
    state.init.hAOAcqTasks(end+1) = state.init.eom.hAO;
end

if state.init.eom.pockelsOn || any(cellfun(@(x)~isempty(state.init.(x)) && ~strcmpi(state.init.(x),state.init.mirrorOutputBoardID),{'nextTrigBoardID' 'frameClockBoardID' 'lineClockBoardID' 'pixelClockBoardID'}))       
    %%%VI090509A: Removed %%%%%%%%%%%%%%%
    %     %Export mirror board's output sample clock onto a RTSI terminal to which it can be directly routed. The Pockels board, if any, will use this signal as its clock.
    %     state.init.hDAQmx.connectTerms(['/' state.init.mirrorOutputBoardID '/ao/SampleClock'], ...
    %         ['/' state.init.mirrorOutputBoardID '/' state.init.outputBoardClockTerminal]);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%VI090509A, VI091009, VI091309
    state.init.hDAQmx.connectTerms(['/' state.init.mirrorOutputBoardID '/20MHzTimebase'], ... %Export clock from mirror/acquisition board on its RTSI7
        ['/' state.init.mirrorOutputBoardID '/RTSI7']);
end

if state.init.eom.pockelsOn
    setTimebaseSource(state.init.eom.hAO);

    for i = 1:state.init.eom.numberOfBeams
        if ~isempty(state.init.hAIPhotodiode{i})
            setTimebaseSource(state.init.hAIPhotodiode{i});
        end
    end
    %deviceFamilies = unique(deviceFamilies);
end

% %%VI071510A, VI071410C: Import master timebase to clock export board(s), as required
% clockExportTasks = {state.init.hFrameClkCtr, state.init.hLineClkCtr, state.init.hPixelClkCtr};
% clockExportTasks(cellfun(@isempty, clockExportTasks)) = [];
% for i = 1:length(clockExportTasks)
%     hCtr = clockExportTasks{i};
%     if ~strcmpi(hCtr.deviceNames{1}, state.init.mirrorOutputBoardID)
%         hCtr.channels.set('ctrTimebaseSrc', 'RTSI7', 'ctrTimebaseRate', 20e6);
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

return;

%--------------------------------------------------------------------------
function setTimebaseSource(hTask)                
%%%VI071410B%%%%%%%%%%

global state

newDeviceFamilies = {'DAQmx_Val_MSeriesDAQ' 'DAQmx_Val_XSeriesDAQ'}; %Tried using refClkTimebase for these devices, but ran into some issue (can't rember what right now) -- Vijay Iyer 9/13/09
oldDeviceFamilies = {'DAQmx_Val_ESeriesDAQ' 'DAQmx_Val_SSeriesDAQ' 'DAQmx_Val_BSeriesDAQ' 'DAQmx_Val_AOSeries'};
deviceNames = hTask.deviceNames;

%No need to synchronize Tasks whose devices are all same same as mirrorOutputBoardID
if all(strcmpi(deviceNames, state.init.mirrorOutputBoardID))
    return;
end

%We do not handle case where Tasks have some devices on mirrorOutputBoardID, and others not
if any(strcmpi(deviceNames, state.init.mirrorOutputBoardID))
    error(['The Task ''' hTask.taskName ''' did not meet at least one the following 3 requirements: 1) All devices should belong to M/X/AO/B/S/E series, 2) All devices must belong to similar device families, e.g. M/X series or AO/B/S/E series, and 3) either all or none of devices should be the same board as primary acquisition/mirror board.']);
end

%Determine device familiies
deviceFamilies = getDeviceFamilies(deviceNames);

if all(ismember(deviceFamilies, newDeviceFamilies))
    hTask.set('sampClkTimebaseRate', 20e6, 'sampClkTimebaseSrc', 'RTSI7');
elseif all(ismember(deviceFamilies, oldDeviceFamilies))
    hTask.set('masterTimebaseRate', 20e6, 'masterTimebaseSrc', 'RTSI7'); %This works if there's /only/ an AO Task running
else
    error(errMsg);
end
return;

%--------------------------------------------------------------------------
function deviceFamilies = getDeviceFamilies(deviceNames)
%A single deviceFamily string, or cell array of unique deviceFamily strings, associated with supplied cell array of deviceNames

import dabs.ni.daqmx.*

deviceFamilies = cell(length(deviceNames),1);
for j=1:length(deviceNames)
    hDevice = Device(deviceNames{j});
    %hDevice = Device.getByName(deviceNames{j});
    deviceFamilies{j} =  hDevice.productCategory;
end
deviceFamilies = unique(deviceFamilies);

if length(deviceFamilies) == 1
    deviceFamilies = deviceFamilies{1};
end
return;
