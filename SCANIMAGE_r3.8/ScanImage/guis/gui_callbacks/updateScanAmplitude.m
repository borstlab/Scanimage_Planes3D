function updateScanAmplitude(handle)
%% function updateScanAmplitude(handle)
% Compute scanAmplitudeFast/Slow values (voltage levels at edge of scan angular range) from INI-defined vars
%
%% NOTES
%   scanAmplitudeFast/Slow are no longer adjustable during SI runtime -- they depend strictly on INI vars%   
%   Function was originally written as an INI-var callback, and remains in that form.
%
%% CHANGES
% VI110210A: Rename state.acq.scanAmplitudeX/Y to state.acq.scanAngularRangeX/Y -- Vijay Iyer 11/2/10
% VI110310A: Restore state.internal.scanAmplitudeX/Y to being an amplitude (i.e. half peak-peak), not a range -- Vijay Iyer 11/3/10
%
%% CREDITS
%   Created 10/26/09, by Vijay Iyer
%% ******************************************************************
global state gh

%Update 'internal' scanAmplitude values
state.internal.scanAmplitudeFast = ((state.init.scanAngularRangeReferenceFast)/2) * state.init.voltsPerOpticalDegree; %VI110310A/VI110210A %DEQ20110520 removing '* state.acq.scanAngleMultiplierFast'
state.internal.scanAmplitudeSlow = ((state.init.scanAngularRangeReferenceSlow)/2) * state.init.voltsPerOpticalDegree; %VI110310A/VI110210A %DEQ20110520 removing '* state.acq.scanAngleMultiplierSlow'


    




        
        