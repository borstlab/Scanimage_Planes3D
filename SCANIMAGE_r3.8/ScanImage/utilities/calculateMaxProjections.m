function calculateMaxProjections
global state gh
%Do ANALYSIS and display images if doing max projections....
%% CHANGES
%   VI091009A: No longer specify EraseMode upon update of data. This is determined a priori elsewhere (currently in makeImageFigures())
%% ************************************************


if (state.acq.numberOfFrames == 1 | state.acq.averaging == 1) & state.acq.numberOfChannelsMax > 0 
%     if state.internal.keepAllSlicesInMemory % BSMOD 1/18/2
%         position = state.internal.zSliceCounter + 1;
%     else
        position = 1;
%     end
    for channelCounter = 1:state.init.maximumNumberOfInputChannels
        if getfield(state.acq, ['maxImage' num2str(channelCounter)]) ...		% If max is on and ...
                & getfield(state.acq,['acquiringChannel' num2str(channelCounter)])	% channel is on
            if	state.internal.zSliceCounter==1	%TPMOD 2/28/02
                if state.acq.maxMode==0
                    state.acq.maxData{channelCounter} = state.acq.acquiredData{1}{channelCounter}(:,:,position);
                else
                    state.acq.maxData{channelCounter} = double(state.acq.acquiredData{1}{channelCounter}(:,:,position));
                end
            else
                if state.acq.maxMode==0
                    state.acq.maxData{channelCounter} = max(state.acq.acquiredData{1}{channelCounter}(:,:,position), ...
                        state.acq.maxData{channelCounter});
                else
                    state.acq.maxData{channelCounter} = ...
                        (double(state.acq.acquiredData{1}{channelCounter}(:,:,state.internal.zSliceCounter)) + ...
                        (state.internal.zSliceCounter - 1) *state.acq.maxData{channelCounter})/(state.internal.zSliceCounter);	
                    %  BSMOD 1/18/2 eliminated reliance on position for above 2 lines
                end					
            end
            % Displays the current Max images on the screen as they are acquired.
            set(state.internal.maximagehandle(channelCounter), 'CData', ... %VI091009A
                uint16(state.acq.maxData{channelCounter})); 	
        end
    end
    drawnow;	
end