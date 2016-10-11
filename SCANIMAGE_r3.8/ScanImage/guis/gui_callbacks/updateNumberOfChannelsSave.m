function updateNumberOfChannelsSave(handle)
global state

state.acq.numberOfChannelsSave = (state.acq.savingChannel1 + state.acq.savingChannel2 + state.acq.savingChannel3 + state.acq.savingChannel4);