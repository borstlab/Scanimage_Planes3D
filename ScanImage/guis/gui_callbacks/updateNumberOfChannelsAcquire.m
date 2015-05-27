function updatenumberOfChannelsAcquire(handle)
global state

state.acq.numberOfChannelsAcquire = (state.acq.acquiringChannel1 + state.acq.acquiringChannel2 + state.acq.acquiringChannel3 + state.acq.acquiringChannel4); %VI102309A