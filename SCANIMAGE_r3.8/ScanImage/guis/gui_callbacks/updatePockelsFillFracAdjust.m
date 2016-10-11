function updatePockelsFillFracAdjust(handle)
%% function updatePockelsFillFracAdjust(handle)
% Callback function that handles update to the Pockels fill frac adjust value 
%
%% NOTES
%   Being an INI-named callback allows this to be called during either a GUI control or CFG/USR file loading event
%
%% CHANGES
%   VI041609A: Compute minAOPeriodIncrement here, rather than relying on state.internal.minAOPeriodIncrement -- Vijay Iyer 4/16/09
%
%% CREDITS
%   Created 1/31/09, by Vijay Iyer
%% ******************************************************************

global state

%Constrain value to valid values 
minAOPeriodIncrement = (1/state.internal.baseOutputRate); %VI041609A
cmdIncrement = 2 * minAOPeriodIncrement * 1e6; %Allow steps of 2 AO sampling periods %VI041609A
state.internal.eom.pockelsFillFracAdjustGUI = round(state.internal.eom.pockelsFillFracAdjustGUI / cmdIncrement) * cmdIncrement;
updateGUIByGlobal('state.internal.eom.pockelsFillFracAdjustGUI');

%Update underlying value
state.acq.pockelsFillFracAdjust = 1e-6 * state.internal.eom.pockelsFillFracAdjustGUI;


