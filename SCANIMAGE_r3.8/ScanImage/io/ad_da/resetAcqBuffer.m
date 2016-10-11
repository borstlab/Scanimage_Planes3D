function resetAcqBuffer()
%% function function resetAcqBuffer()
% Resets the acquisition buffer 
%

global state

for i=1:length(state.acq.acquiredData)
    for j=1:state.init.maximumNumberOfInputChannels
        state.acq.acquiredData{i}{j}(:) = 0;
    end
end

