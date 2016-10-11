function [fillFraction, msPerLine] = decodeFillFractionGUI(fillFractionGUI)
%DECODEFILLFRACTIONGUI Extracts fill fraction and ms/line values from fillFractionGUI value (index)

global state

incrementMultiplier = state.internal.linePeriodIncrementMultipliers(fillFractionGUI);

incrementMultiplierTotal = round(incrementMultiplier * state.internal.linePeriodIncrementFactor);  %Compute total multiplier of minimum increment, which must be an integer
 
msPerLine = state.internal.nominalMsPerLine + incrementMultiplierTotal *  si_getMinLinePeriodIncrement() * 1e3;
fillFraction = state.internal.activeMsPerLine / msPerLine;




