function out = startPMTOffsets(forceAllChan)
% Function that will start the aiPMTOffsets DAQ device.
%
%% SYNTAX
%   forceAllChan: <Default=false> If true, offset will be determined for all channels; otherwise, offsets determined only for currently active channels
%
%% CHANGES
%   VI090109A: Changes to use new DAQmx interface -- Vijay Iyer 9/1/09
%   VI091109A: Make use of shared hAI Task, rather than separate Task for the PMT offsets measurement. Use polling and directly call calculatePMTOffsets() -- rather than using a DoneEvt callback.
%   VI100710A: Reduce number of lines acquired PMT offset determination; move calculatePMTOffsets() into this file as subfunction -- Vijay Iyer 10/7/10
%   VI101910A: Account for possibility of photodiode channels on primary board, limiting number of PMT inputs -- Vijay Iyer 9/13/09
%
%% *************************************************
global state

%Process input arguments
if nargin == 0
    forceAllChan = false;
end

out=0;

offsetNumLines = 1; %VI100710A

%status=state.internal.statusString;
setStatusString('Reading PMT offsets...');

%%%VI090109A: Removed%%%%%%%%%%%
% start(state.init.aiPMTOffsets);
% while strcmp(state.init.aiPMTOffsets.Running, 'On')
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI090109A, VI091109A%%%%%%%
everyNSamples = state.init.hAI.everyNSamples;
readChansToRead = get(state.init.hAI,'readChannelsToRead');

state.init.hAI.everyNSamples = []; %Deactivates EveryN callback
if forceAllChan %Configure AI Task to read data from /all/ channels regardless of whether active or not
    channelsToAcquire = '';
    for i=1:state.init.maximumNumberOfInputChannels - length(state.init.primaryBoardPhotodiodeChans) %VI091309A
        channelsToAcquire = [channelsToAcquire state.init.hAI.channels(i).chanName ',']; %#ok<AGROW>        
    end
    
    state.init.hAI.set('readChannelsToRead',channelsToAcquire);
end


state.init.hAI.disableStartTrig();

numSampsToAcquire = offsetNumLines * state.acq.samplesAcquiredPerLine;
state.init.hAI.set('readOverWrite','DAQmx_Val_OverwriteUnreadSamps');
state.init.hAI.set('readRelativeTo','DAQmx_Val_MostRecentSamp');
state.init.hAI.set('readOffset',-numSampsToAcquire);


try 
    state.init.hAI.start();
    while state.init.hAI.get('readTotalSampPerChanAcquired') < numSampsToAcquire; %VI100710A
        pause(.01);
    end
    offsetData = state.init.hAI.readAnalogData(numSampsToAcquire, 'native'); %VI100710A
    
    state.init.hAI.stop(); %VI100710A
    
    calculatePMTOffsets(forceAllChan,offsetData); %VI091109A

catch ME %#ok<NASGU>
    fprintf(2,'WARNING: Unable to read/determine PMT offsets\n');
    restoreAITask(everyNSamples,readChansToRead);
    return;
end

restoreAITask(everyNSamples,readChansToRead);

%setStatusString(status);
out=1;

% if nargout == 1
% 	varargout{1} = out;
% end

function restoreAITask(everyNSamples,readChansToRead)

global state

state.init.hAI.reset('readOverWrite');
state.init.hAI.reset('readRelativeTo');
state.init.hAI.reset('readOffset');

if ~isempty(everyNSamples)
    state.init.hAI.everyNSamples = everyNSamples;
end

state.init.hAI.readChannelsToRead = readChansToRead;

return;


%%%VI100710A%%%%
function calculatePMTOffsets(forceAllChan,offsetData)

global state 

try
    %offsetData = addForPmtOffsets(offsetData, state.acq.binFactor); % Adds just like acquisition
    
    readChannelCounter = 1;
    
    for channelCounter = 1:(state.init.maximumNumberOfInputChannels - length(state.init.primaryBoardPhotodiodeChans)) %VI101910A
        
        if forceAllChan || state.acq.(['acquiringChannel' num2str(channelCounter)]) % if statement only gets executed when there is a channel to acquire.
           
            state.acq.(['pmtOffsetChannel' num2str(channelCounter)]) = mean(offsetData(:,readChannelCounter));
            state.acq.(['pmtOffsetStdDevChannel' num2str(channelCounter)]) = std(double(offsetData(:,readChannelCounter)));
            %			eval(['state.acq.pmtOffsetMeanVarChannel' num2str(channelCounter) ...
            %					' = state.acq.pmtOffsetChannel' num2str(channelCounter) '/state.acq.pmtOffsetStdDevChannel' num2str(channelCounter) ';']);
                 
            updateHeaderString(['state.acq.pmtOffsetChannel' num2str(channelCounter)]);
            updateHeaderString(['state.acq.pmtOffsetStdDevChannel' num2str(channelCounter)]);
            
            updateGUIByGlobal(sprintf('state.acq.pmtOffsetChannel%d',channelCounter));
            
            readChannelCounter = readChannelCounter + 1;
            
        end
    end
catch ME
    ME.throwAsCaller();
end
%%%%%%%%%%%%%%%%%%%




