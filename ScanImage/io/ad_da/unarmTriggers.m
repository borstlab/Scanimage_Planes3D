function unarmTriggers()
%% function unarmTriggers()
% Unarms any triggers configured for finished acuqisition, freeing any relevant resources.
%
%% CREDITS
%   Created 9/19/09, by Vijay Iyer
%% ****************************************************************************

global state

state.init.hStartTrigCtr.stop();

if ~isempty(state.init.hNextTrigCtr)
    state.init.hNextTrigCtr.stop();
end

