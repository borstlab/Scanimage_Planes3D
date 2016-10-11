function updateFrameRate()
global state

state.acq.frameRate = 1 / (state.acq.msPerLine * 1e-3 * state.acq.linesPerFrame);
state.acq.frameRateGUI = str2num(num2str(state.acq.frameRate,'%4.2f')); %Format the value to 2 decimal points.
updateGUIByGlobal('state.acq.frameRateGUI');