function setUndo
global state

state.acq.lastROIForUndo=[state.acq.scanShiftFast state.acq.scanShiftSlow state.acq.scanRotation ...
    state.acq.zoomFactor];