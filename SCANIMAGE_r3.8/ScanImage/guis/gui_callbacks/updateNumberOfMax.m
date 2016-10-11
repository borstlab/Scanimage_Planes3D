function updateNumberOfMax(handle)
global state

state.acq.numberOfChannelsMax = (state.acq.maxImage1 + state.acq.maxImage2 + state.acq.maxImage3 + state.acq.maxImage4);