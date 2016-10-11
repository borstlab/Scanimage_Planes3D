function [fillFraction, msPerLine] = applyIncrementMultiplier(incrementMultiplier)
%COMPUTEMSPERLINE Computes fill fraction and msPerLine values from increment-multiplier value
%
%% CREDITS
%   Created 1/21/09, by Vijay Iyer
%% ******************************************

global state

msPerLine = state.internal.nominalMsPerLine + incrementMultiplier * state.internal.linePeriodIncrement * 1e3;
fillFraction = state.internal.activeMsPerLine / msPerLine;




