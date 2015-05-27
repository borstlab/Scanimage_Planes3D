function applyChannelSettings
%% function applyChannelSettings
% Handle when Channel GUI is changed
%
%% CHANGES
%   VI082709A: Use new DAQmx interface -- Vijay Iyer 8/27/09
%   VI091309A: Account for possibility of photodiode channels on primary board, limiting number of PMT inputs -- Vijay Iyer 9/13/09
%   VI120511A: Update PMT Offset measurements for all channels when any channel setting is changed
%% ********************

global state

%Create vectors, from the separated scalar variables, indicating which channels are acquiring/saving/imaging/maxProjectioning
state.acq.acquiringChannel=[];
state.acq.savingChannel=[];
state.acq.imagingChannel=[];
state.acq.maxImage=[];
for i = 1:state.init.maximumNumberOfInputChannels
	state.acq.acquiringChannel = [state.acq.acquiringChannel eval(['state.acq.acquiringChannel' num2str(i)])];
	state.acq.savingChannel	= [state.acq.savingChannel eval(['state.acq.savingChannel' num2str(i)])];
	state.acq.imagingChannel	= [state.acq.imagingChannel eval(['state.acq.imagingChannel' num2str(i)])];
	state.acq.maxImage = [state.acq.maxImage eval(['state.acq.maxImage' num2str(i)])];
end

%%%REMOVED (VI082709A)%%%%%%%%%%%%%%%
% deleteAIObjects;
% setupAIObjects_Common;					% creates AI Objects
% addChannelsToAIObjects;					% adds the appropriate channels to the AI Object
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI082709A%%%%%%%%%%
PMTAITasks =  {state.init.hAI}; %VI091109A
cellfun(@selectTaskChannels,PMTAITasks);

    function selectTaskChannels(task)
        channelsToAcquire = '';
        for i=1:state.init.maximumNumberOfInputChannels - length(state.init.primaryBoardPhotodiodeChans) %VI091309A
            if state.acq.acquiringChannel(i)
                channelsToAcquire = [channelsToAcquire task.channels(i).chanName ',']; %#ok<AGROW>
            end
            %Set voltage range based on specified setting, regardless of whether acquiring or not
            voltageRange = state.acq.(['inputVoltageRange' num2str(i)]);
            task.channels(i).set('rngLow',-voltageRange, 'rngHigh', voltageRange);
        end
        
        %Remove trailing comma (not sure if req'd)
        if ~isempty(channelsToAcquire) 
           channelsToAcquire(end) = []; 
        end
        
        %Update the Task's channels...
        task.set('readChannelsToRead',channelsToAcquire);

    end
%%%%%%%%%%%%%%%%%%%

startPMTOffsets(true); %VI120511A

updateClim;
applyConfigurationSettings;
enableLUTControls();		% update LUT window

state.internal.channelChanged=0;
updateHeaderString('state.acq.numberOfChannelsAcquire')
updateHeaderString('state.acq.numberOfChannelsSave')

end