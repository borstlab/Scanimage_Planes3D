function  updateNumAvgFramesDisplay(varargin)
%% function  updateNumAvgFramesDisplay(varargin)
%Shared callback logic for handling updates to value specifying the number of frames to average for display (rolling average)

global state

state.acq.averagingDisplay = (state.acq.numAvgFramesDisplay > 1);

if state.acq.lockAvgFrames && (state.acq.numAvgFramesDisplay ~= state.acq.numAvgFramesSaveGUI)
    state.acq.numAvgFramesSaveGUI = state.acq.numAvgFramesDisplay;
    updateGUIByGlobal('state.acq.numAvgFramesSaveGUI','Callback',1);
else
    preallocateMemory();
end
