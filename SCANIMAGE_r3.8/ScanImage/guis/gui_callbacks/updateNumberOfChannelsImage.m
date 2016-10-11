function updateNumberOfChannelsImage(handle)
global state

state.acq.numberOfChannelsImage = (state.acq.imagingChannel1 + state.acq.imagingChannel2 + state.acq.imagingChannel3 + state.acq.imagingChannel4);