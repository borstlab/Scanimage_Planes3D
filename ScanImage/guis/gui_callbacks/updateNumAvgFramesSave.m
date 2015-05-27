function  updateNumAvgFramesSave(varargin)
%% function  updateNumAvgFramesSave(varargin)
%Shared callback logic for handling updates to value specifying the number of frames to average during saving

global gh state %#ok<NUSED>

c = getGuiOfGlobal('state.acq.numAvgFramesSaveGUI');
hNumAvgSave = eval(c{1});

state.acq.numAvgFramesSave = min(state.acq.numberOfFrames,state.acq.numAvgFramesSaveGUI);

%NOTE - first 2 cases are redundant/identically handled, but they could potentially be handled separately in future
if state.acq.numberOfFrames < state.acq.numAvgFramesSaveGUI
    state.acq.averaging = 0;
    state.acq.numAvgFramesSave = 1;
    set(hNumAvgSave,'BackgroundColor',[1 .75 .75]);
elseif rem(state.acq.numberOfFrames,state.acq.numAvgFramesSaveGUI) ~= 0  %# frames not an integer multiple of the num averaged frames 
    state.acq.averaging = 0;
    state.acq.numAvgFramesSave = 1;
    set(hNumAvgSave,'BackgroundColor',[1 .75 .75]);
else
    state.acq.averaging = 1;
    state.acq.numAvgFramesSave = state.acq.numAvgFramesSaveGUI;
    set(hNumAvgSave,'BackgroundColor','white');
end

if state.acq.lockAvgFrames &&  state.acq.numAvgFramesDisplay ~= state.acq.numAvgFramesSaveGUI
    state.acq.numAvgFramesDisplay = state.acq.numAvgFramesSaveGUI;
    updateGUIByGlobal('state.acq.numAvgFramesDisplay','Callback',true);
else
    preallocateMemory();
end

    