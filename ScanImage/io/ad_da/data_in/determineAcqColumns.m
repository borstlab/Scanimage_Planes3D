function [startColumn endColumn] = determineAcqColumns()
%DETERMINEACQCOLUMNS Determine column indices for image data corresponding to interval in whcih to acquire data

global state

startColumn = round((state.acq.acqDelay + state.acq.scanDelay) * state.acq.inputRate) + 1;
endColumn = startColumn + (state.acq.samplesAcquiredPerLine-1);
