function updateMaxLUTValue(handle)
%% function updateMaxLUTValue(handle)
%   Handles updates to properties affecting the maximum LUT value
%
%% NOTES
%   Function is an INI-file callback, so it is invoked either upon adjusting relevant GUI controls or loading a USR/CFG file
%
%   At moment, 2 varying values affect MaxLUTValue: state.acq.binFactor & state.internal.averageSamples
% 
%% CREDITS
%   Created 11/23/10, by Vijay Iyer
%% ************************************************

global state gh

if state.internal.averageSamples
    maxLUTValue = 2^(state.acq.inputBitDepth - 1);
else
    maxLUTValue = 2^(state.acq.inputBitDepth - 1) * state.acq.binFactor;
end
state.internal.maxLUTValue = maxLUTValue;

for i=1:state.init.maximumNumberOfInputChannels
    hWhite = gh.imageControls.(sprintf('whiteSlideChan%d',i));
    hBlack = gh.imageControls.(sprintf('blackSlideChan%d',i));
    
    %Ensure values don't exceed new maximum value
    if get(hWhite,'Value') > maxLUTValue
        state.internal.(sprintf('highPixelValue%d',i)) = maxLUTValue;
        updateGUIByGlobal(sprintf('state.internal.highPixelValue%d',i));
    end
    
    if get(hBlack,'Value') > maxLUTValue
        state.internal.(sprintf('lowPixelValue%d',i)) = maxLUTValue;
        updateGUIByGlobal(sprintf('state.internal.lowPixelValue%d',i));
    end
    
    %Update slider Max values
    set(hWhite,'Max',maxLUTValue);
    set(hBlack,'Max',maxLUTValue-1);

end
