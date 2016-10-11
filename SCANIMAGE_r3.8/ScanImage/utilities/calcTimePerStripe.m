function out=calcTimePerStripe
global state
% Benchmarking routine for ScanImage...
% Must uncomment the lines that are placed in the following functions:
% makeStripe & executeFocusCallback
%
% Then set the state.internal.numberOfFocusFrames=20.
% Then hit focus and let it run till it stops for the particular configuration.
% Then execute this function.
%
% This function outputs the Realtime fraction which is the time it takes to acquire 
% a stripe of data (time below) divided by the data for the elapsed times recorded in the
% software and stored in state.time.

% Percent Realtime...
time=(1e-3 * state.acq.msPerLine * (state.acq.linesPerFrame/state.internal.numberOfStripes)); %VI012109A
normaltime=time*state.time(2:end).^-1;
out=[mean(normaltime) std(normaltime)];